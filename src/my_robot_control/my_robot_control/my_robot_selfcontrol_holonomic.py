import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

def clamp(x, lo, hi):
    return max(lo, min(x, hi))

def normalize_deg(deg):
    """Normalize angle to [-180, 180) degrees"""
    # use modulo to avoid edge cases
    nd = ((deg + 180.0) % 360.0) - 180.0
    return nd

class RobotSelfControlHolonomic(Node):
    def __init__(self):
        super().__init__("robot_selfcontrol_holonomic_fixed")

        # Parameters (tweak these)
        self.declare_parameter("distance_limit", 0.30)   # emergency trigger
        self.declare_parameter("distance_safe", 0.50)    # hysteresis release
        self.declare_parameter("vf_nominal", 0.20)       # nominal forward
        self.declare_parameter("vy_nominal", 0.18)       # nominal side speed
        self.declare_parameter("back_time", 0.5)         # seconds to back up if too close
        self.declare_parameter("avoid_forward", 0.06)    # small forward while sliding
        self.declare_parameter("smoothing_alpha", 0.35)  # velocity smoothing (0..1)
        self.declare_parameter("time_to_stop", 60.0)

        self.d_limit = float(self.get_parameter("distance_limit").value)
        self.d_safe = float(self.get_parameter("distance_safe").value)
        self.vf_nom = float(self.get_parameter("vf_nominal").value)
        self.vy_nom = float(self.get_parameter("vy_nominal").value)
        self.back_time = float(self.get_parameter("back_time").value)
        self.avoid_forward = float(self.get_parameter("avoid_forward").value)
        self.alpha = float(self.get_parameter("smoothing_alpha").value)
        self.time_to_stop = float(self.get_parameter("time_to_stop").value)

        # publishers/subscribers/timer
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.sub = self.create_subscription(LaserScan, "/scan", self.laser_callback, 10)
        self.timer = self.create_timer(0.05, self.timer_callback)

        # state
        self.mode = "FORWARD"  # FORWARD, AVOID_LEFT, AVOID_RIGHT, BACKING
        self.cmd = Twist()
        self.cmd.linear.x = self.vf_nom
        self.cmd.linear.y = 0.0
        self.cmd.angular.z = 0.0

        # initialize smoothed_cmd to current cmd to avoid spikes
        self.smoothed_cmd = Twist()
        self.smoothed_cmd.linear.x = self.cmd.linear.x
        self.smoothed_cmd.linear.y = self.cmd.linear.y
        self.smoothed_cmd.angular.z = self.cmd.angular.z

        self.backing_until = None
        self.start_time = self.get_clock().now().nanoseconds * 1e-9
        self.last_log = 0.0
        self.stopping = False

        self.get_logger().info("Self-control holonomic (fixed) started")

    def timer_callback(self):
        if self.stopping:
            return

        # publish smoothed command (exponential smoothing)
        # linear.x
        prev_x = self.smoothed_cmd.linear.x
        target_x = self.cmd.linear.x
        self.smoothed_cmd.linear.x = prev_x + self.alpha * (target_x - prev_x)

        # linear.y
        prev_y = self.smoothed_cmd.linear.y
        target_y = self.cmd.linear.y
        self.smoothed_cmd.linear.y = prev_y + self.alpha * (target_y - prev_y)

        # angular.z
        prev_w = self.smoothed_cmd.angular.z
        target_w = self.cmd.angular.z
        self.smoothed_cmd.angular.z = prev_w + self.alpha * (target_w - prev_w)

        self.pub.publish(self.smoothed_cmd)

        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.start_time > self.time_to_stop:
            self.get_logger().info("time_to_stop reached — stopping")
            self.stop_robot()
            rclpy.try_shutdown()
            return

        if now - self.last_log > 1.0:
            self.get_logger().info(f"MODE:{self.mode} CMD vx={self.smoothed_cmd.linear.x:.3f} vy={self.smoothed_cmd.linear.y:.3f} w={self.smoothed_cmd.angular.z:.3f}")
            self.last_log = now

    def laser_to_sectors(self, scan):
        """Return minima for several useful sectors (meters). 
           Sectors in degrees (robot frame):
           front: -15..+15, front-left: +15..+60, front-right: -60..-15, left: +60..+120, right: -120..-60
        """
        angle_min = scan.angle_min
        a_inc = scan.angle_increment
        ranges = scan.ranges

        # initialize with large distances
        INF = float("inf")
        sectors = {
            "front":  INF,
            "front_left": INF,
            "front_right": INF,
            "left": INF,
            "right": INF,
            "closest": INF,
            "closest_angle": 0.0
        }

        for i, r in enumerate(ranges):
            if not math.isfinite(r):
                continue
            if r <= 0.0 or r < scan.range_min or r > scan.range_max:
                continue
            angle = angle_min + i * a_inc
            deg = math.degrees(angle)
            deg = normalize_deg(deg)  # robust normalization

            # update closest
            if r < sectors["closest"]:
                sectors["closest"] = r
                sectors["closest_angle"] = deg

            # sectors
            if -15 <= deg <= 15:
                sectors["front"] = min(sectors["front"], r)
            if 15 < deg <= 60:
                sectors["front_left"] = min(sectors["front_left"], r)
            if -60 <= deg < -15:
                sectors["front_right"] = min(sectors["front_right"], r)
            if 60 < deg <= 120:
                sectors["left"] = min(sectors["left"], r)
            if -120 <= deg < -60:
                sectors["right"] = min(sectors["right"], r)

        return sectors

    def laser_callback(self, scan: LaserScan):
        # get sector minima
        s = self.laser_to_sectors(scan)

        closest = s["closest"]
        closest_angle = s["closest_angle"]
        front = s["front"]
        fl = s["front_left"]
        fr = s["front_right"]
        left = s["left"]
        right = s["right"]

        now = self.get_clock().now().nanoseconds * 1e-9

        # Emergency backing when something is inside absolute limit
        if closest < self.d_limit:
            if self.mode != "BACKING":
                self.get_logger().warn(f"EMERGENCY: closest {closest:.3f}m at {closest_angle:.0f}°. BACKING for {self.back_time}s")
            self.mode = "BACKING"
            self.backing_until = now + self.back_time

        # If backing, keep backing until backing_until
        if self.mode == "BACKING":
            self.cmd.linear.x = -self.vf_nom
            self.cmd.linear.y = 0.0
            self.cmd.angular.z = 0.0
            if self.backing_until and now >= self.backing_until:
                # after backing try to pick avoidance direction
                # prefer side with larger gap among left/right/front-left/front-right
                # use large default when INF
                left_gap = min([d for d in [left, fl] if d != float("inf")] + [10.0])
                right_gap = min([d for d in [right, fr] if d != float("inf")] + [10.0])
                if left_gap > right_gap:
                    self.mode = "AVOID_LEFT"   # obstacle on right => move left
                else:
                    self.mode = "AVOID_RIGHT"
                self.get_logger().info(f"BACKING done — next mode {self.mode}")
            return

        # Normal avoidance decision (not backing)
        # Determine frontal_relevance robustly
        frontal_candidates = [d for d in [front, fl, fr] if d != float("inf")]
        if frontal_candidates:
            frontal_relevance = min(frontal_candidates)
        else:
            frontal_relevance = 10.0

        # Hysteresis: if currently avoiding, require d_safe to return to FORWARD
        if self.mode in ["AVOID_LEFT", "AVOID_RIGHT"]:
            if frontal_relevance > self.d_safe:
                self.mode = "FORWARD"

        # If too close in front but not closer than d_limit (handled above), decide side avoidance
        if frontal_relevance < self.d_safe and frontal_relevance >= self.d_limit:
            # pick side: if front_left is smaller (closer) than front_right => move right
            # handle INF properly
            fl_val = fl if fl != float("inf") else 10.0
            fr_val = fr if fr != float("inf") else 10.0
            if fl_val < fr_val:
                self.mode = "AVOID_RIGHT"
            else:
                self.mode = "AVOID_LEFT"

        # ANTI-GET-STUCK: if left and right both have close minima, do tie-break by angle
        left_hits = [d for d, a in [(left, None)] if left != float("inf")]  # placeholder (we check sectors separately below)
        # collect actual left and right lists from scan for more robust tie-break
        # we'll approximate by using fl/left and fr/right
        left_min = min([x for x in [fl, left] if x != float("inf")] + [10.0])
        right_min = min([x for x in [fr, right] if x != float("inf")] + [10.0])
        if abs(left_min - right_min) < 0.05 and left_min < 10.0 and right_min < 10.0:
            # tie situation — prefer moving away from side that is more frontal (smaller |angle|)
            # use the closest angles from previously computed sectors via scan: we have only distances and not angles per sector,
            # but we can use closest_angle to bias direction when closest is near left or right
            ca = normalize_deg(closest_angle)
            # if closest angle is positive (left side) -> prefer move right
            if ca > 0:
                self.mode = "AVOID_RIGHT"
            else:
                self.mode = "AVOID_LEFT"

        # Side-aware holonomic rule:
        # If the global closest obstacle is within the safety distance and lies clearly
        # on one side (outside the frontal band), prefer to move to the opposite side.
        # Example: closest_angle < 0 (right side) -> move left (AVOID_LEFT).
        if closest < self.d_safe:
            ca = normalize_deg(closest_angle)
            # consider it a side obstacle if angle is outside the small frontal band
            if abs(ca) > 15.0:
                if ca < 0:
                    self.mode = "AVOID_LEFT"
                else:
                    self.mode = "AVOID_RIGHT"

        # Compose cmd from mode, with forward scaling
        # forward_scale goes from 0 at d_limit to 1 at (d_safe) (linear)
        denom = max(1e-3, (self.d_safe - self.d_limit))
        forward_scale = clamp((frontal_relevance - self.d_limit) / denom, 0.0, 1.0)
        effective_vf = self.vf_nom * forward_scale

        if self.mode == "FORWARD":
            self.cmd.linear.x = effective_vf
            self.cmd.linear.y = 0.0
            self.cmd.angular.z = 0.0

        elif self.mode == "AVOID_LEFT":
            # obstacle left => move right. reduce forward to avoid collision
            self.cmd.linear.x = clamp(self.avoid_forward * forward_scale + 0.02, 0.0, self.vf_nom * 0.6)
            # obstacle left -> move right => vy negative (depending on robot convention)
            self.cmd.linear.y = -self.vy_nom
            self.cmd.angular.z = 0.0

        elif self.mode == "AVOID_RIGHT":
            self.cmd.linear.x = clamp(self.avoid_forward * forward_scale + 0.02, 0.0, self.vf_nom * 0.6)
            # obstacle right -> move left => vy positive
            self.cmd.linear.y = self.vy_nom
            self.cmd.angular.z = 0.0

        # Safety clamp: if any commanded forward is positive while closest frontal is dangerously close, force reduce
        if closest < (self.d_limit + 0.05) and self.cmd.linear.x > 0.0:
            self.cmd.linear.x = 0.0

        # final safety caps
        self.cmd.linear.x = clamp(self.cmd.linear.x, -self.vf_nom, self.vf_nom)
        self.cmd.linear.y = clamp(self.cmd.linear.y, -self.vy_nom, self.vy_nom)
        self.cmd.angular.z = clamp(self.cmd.angular.z, -1.0, 1.0)

    def stop_robot(self):
        stop = Twist()
        self.pub.publish(stop)
        self.stopping = True
        self.get_logger().info("stopped")


def main(args=None):
    rclpy.init(args=args)
    node = RobotSelfControlHolonomic()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()