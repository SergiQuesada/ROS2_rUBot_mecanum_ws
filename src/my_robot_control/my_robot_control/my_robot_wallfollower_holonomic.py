import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower_node')

        # Parameters
        self.declare_parameter('distance_limit', 0.5)    # desired distance to right wall
        self.declare_parameter('forward_speed', 0.20)    # linear speed
        self.declare_parameter('turn_speed', 0.40)       # angular speed
        self.declare_parameter('time_to_stop', 30.0)     # auto-stop
        self.declare_parameter('tolerance', 0.05)        # band around base_distance (RIGHT)

        self.base_distance = float(self.get_parameter('distance_limit').value)
        self.v_lin = float(self.get_parameter('forward_speed').value)
        self.v_ang = float(self.get_parameter('turn_speed').value)
        self.time_to_stop = float(self.get_parameter('time_to_stop').value)
        self.tol = float(self.get_parameter('tolerance').value)

        # Last commanded twist (will be published periodically)
        self.cmd = Twist()

        # ROS 2 entities
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, qos_profile_sensor_data
        )
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timers
        self.info_timer = self.create_timer(1.0, self.log_info)
        self.stop_timer = self.create_timer(0.05, self.stop_watchdog)

        # Periodic cmd_vel publisher at 10 Hz (0.1 s)
        self.cmd_timer = self.create_timer(0.1, self.cmd_publish_timer_cb)

        self._state_action = "Idle"
        self._last_action_logged = None
        self._shutting_down = False

        self.start_time_s = self.get_clock().now().nanoseconds * 1e-9

        self.get_logger().info(
            "WallFollower (HOLONOMIC, 8 sectors) - strafing capable."
        )

    #--------------------------------------------------------------------
    def stop_watchdog(self):
        """Stop the robot after time_to_stop seconds."""
        if self._shutting_down:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.start_time_s >= self.time_to_stop:
            self.get_logger().info("Stopping due to timeout.")
            self.stop()

    #--------------------------------------------------------------------
    def stop(self):
        """Safe stop: set cmd to zero Twist, try to publish once, stop timers."""
        self._shutting_down = True

        # Set last command to zero
        self.cmd = Twist()

        # Try a final publish (publisher may still be valid even if shutdown started)
        try:
            self.publisher.publish(self.cmd)
        except Exception:
            # Context/publisher may already be invalid -> ignore
            pass

        # Cancel timers safely
        for t in [self.info_timer, self.stop_timer, self.cmd_timer]:
            try:
                t.cancel()
            except Exception:
                pass

    #--------------------------------------------------------------------
    def cmd_publish_timer_cb(self):
        """Periodic publisher: send the latest cmd_vel at 10 Hz."""
        if self._shutting_down:
            return

        try:
            self.publisher.publish(self.cmd)
        except Exception:
            # If the context or publisher is invalid, ignore
            pass

    #--------------------------------------------------------------------
    def laser_callback(self, scan):
        """Compute control action from LIDAR and update self.cmd."""
        if self._shutting_down:
            return

        angle_min = math.degrees(scan.angle_min)
        angle_inc = math.degrees(scan.angle_increment)

        FRONT       = []
        FR_RIGHT    = []
        RIGHT       = []
        BACK_RIGHT  = []
        BACK        = []
        BACK_LEFT   = []
        LEFT        = []
        FR_LEFT     = []

        for i, d in enumerate(scan.ranges):
            if not math.isfinite(d):
                continue
            if d < scan.range_min or d > scan.range_max:
                continue

            ang = angle_min + i * angle_inc

            # assign degrees to 8 sectors (bounds approximated to match original code ranges)
            if -22.5 <= ang <= 22.5:
                FRONT.append(d)
            elif -67.5 < ang < -22.5:
                FR_RIGHT.append(d)
            elif -112.5 <= ang <= -67.5:
                RIGHT.append(d)
            elif -157.5 < ang < -112.5:
                BACK_RIGHT.append(d)
            elif ang > 157.5 or ang <= -157.5:
                BACK.append(d)
            elif 112.5 < ang <= 157.5:
                BACK_LEFT.append(d)
            elif 67.5 < ang <= 112.5:
                LEFT.append(d)
            elif 22.5 < ang <= 67.5:
                FR_LEFT.append(d)

        # Minimal distances for all sectors
        min_front      = min(FRONT)      if FRONT      else float('inf')
        min_fr_right   = min(FR_RIGHT)   if FR_RIGHT   else float('inf')
        min_right      = min(RIGHT)      if RIGHT      else float('inf')
        min_back_right = min(BACK_RIGHT) if BACK_RIGHT else float('inf')
        min_back       = min(BACK)       if BACK       else float('inf')
        min_back_left  = min(BACK_LEFT)  if BACK_LEFT  else float('inf')
        min_left       = min(LEFT)       if LEFT       else float('inf')
        min_fr_left    = min(FR_LEFT)    if FR_LEFT    else float('inf')

        twist = Twist()
        action = ""

        # helper: orientation correction using front vs back readings on the same side
        def orient_correction(front_dist, back_dist, sign=1.0):
            if not math.isfinite(front_dist) or not math.isfinite(back_dist):
                return 0.0
            diff = front_dist - back_dist
            return sign * self.v_ang * 0.5 * (diff / (self.base_distance + 1e-6))

        # Choose the closest sector (smallest distance)
        sectors = {
            'FRONT': min_front,
            'FRONT_RIGHT': min_fr_right,
            'RIGHT': min_right,
            'BACK_RIGHT': min_back_right,
            'BACK': min_back,
            'BACK_LEFT': min_back_left,
            'LEFT': min_left,
            'FRONT_LEFT': min_fr_left,
        }

        closest = min(sectors, key=lambda k: sectors[k])
        closest_dist = sectors[closest]

        # Map closest sector to holonomic motion (linear.x forward, linear.y left)
        if closest_dist == float('inf'):
            # If no wall is detected at all, move forward
            twist.linear.x = self.v_lin
            twist.linear.y = 0.0
            twist.angular.z = 0.0
            action = "No wall detected -> FORWARD"

        elif closest == 'FRONT' and closest_dist < self.base_distance:
            twist.linear.x = 0.0
            twist.linear.y = self.v_lin  # strafe left
            twist.angular.z = 0.0
            action = f"FRONT {closest_dist:.2f} m -> STRAFE LEFT"

        elif closest == 'FRONT_RIGHT' and closest_dist < self.base_distance:
            twist.linear.x = self.v_lin * 0.8
            twist.linear.y = self.v_lin * 0.6  # forward-left
            twist.angular.z = 0.0
            action = f"FRONT-RIGHT {closest_dist:.2f} m -> FORWARD-LEFT"

        elif closest == 'RIGHT' and math.isfinite(closest_dist):
            twist.linear.x = self.v_lin
            twist.linear.y = 0.0
            # keep parallel to wall using front-right and back-right
            twist.angular.z = orient_correction(min_fr_right, min_back_right, sign=1.0)
            action = f"RIGHT {closest_dist:.2f} m -> FORWARD, orient corr {twist.angular.z:.2f}"

        elif closest == 'BACK_RIGHT' and closest_dist < self.base_distance:
            twist.linear.x = self.v_lin * 0.8
            twist.linear.y = -self.v_lin * 0.6  # forward-right
            twist.angular.z = 0.0
            action = f"BACK-RIGHT {closest_dist:.2f} m -> FORWARD-RIGHT"

        elif closest == 'BACK' and closest_dist < self.base_distance:
            twist.linear.x = 0.0
            twist.linear.y = -self.v_lin  # strafe right
            twist.angular.z = 0.0
            action = f"BACK {closest_dist:.2f} m -> STRAFE RIGHT"

        elif closest == 'BACK_LEFT' and closest_dist < self.base_distance:
            twist.linear.x = self.v_lin * 0.8
            twist.linear.y = self.v_lin * 0.6  # forward-left
            twist.angular.z = 0.0
            action = f"BACK-LEFT {closest_dist:.2f} m -> FORWARD-LEFT"

        elif closest == 'LEFT' and math.isfinite(closest_dist):
            twist.linear.x = self.v_lin
            twist.linear.y = 0.0
            twist.angular.z = orient_correction(min_fr_left, min_back_left, sign=-1.0)
            action = f"LEFT {closest_dist:.2f} m -> FORWARD, orient corr {twist.angular.z:.2f}"

        elif closest == 'FRONT_LEFT' and closest_dist < self.base_distance:
            twist.linear.x = self.v_lin * 0.8
            twist.linear.y = -self.v_lin * 0.6  # forward-right to escape left-front obstacle
            twist.angular.z = 0.0
            action = f"FRONT-LEFT {closest_dist:.2f} m -> FORWARD-RIGHT"

        # If no action was chosen above (e.g. all distances > base_distance),
        # use a minimal fallback: move forward. This ensures the robot does
        # not remain stopped with message "Stopped (no wall detected)".
        if not action:
            twist.linear.x = self.v_lin
            twist.linear.y = 0.0
            twist.angular.z = 0.0
            action = "Fallback -> FORWARD"

        # Update last commanded twist (periodic timer will publish it)
        self.cmd = twist

        # Logging (only on change)
        if action != self._last_action_logged:
            self.get_logger().info(action if action else "No action (stopped).")
            self._last_action_logged = action

        self._state_action = action if action else "Stopped (no wall detected)"

    #--------------------------------------------------------------------
    def log_info(self):
        if not self._shutting_down:
            self.get_logger().info(self._state_action)

def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop()
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass

        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()