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
        self.declare_parameter('side_speed', 0.18)       # lateral speed for holonomic
        self.declare_parameter('side_kp', 0.8)          # lateral P gain for wall-centering
        self.declare_parameter('time_to_stop', 30.0)     # auto-stop
        self.declare_parameter('tolerance', 0.05)        # band around base_distance (RIGHT)

        self.base_distance = float(self.get_parameter('distance_limit').value)
        self.v_lin = float(self.get_parameter('forward_speed').value)
        self.v_ang = float(self.get_parameter('turn_speed').value)
        self.v_side = float(self.get_parameter('side_speed').value)
        self.side_kp = float(self.get_parameter('side_kp').value)
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
            "WallFollower (RIGHT tol, BACK_RIGHT when closest) - differential drive."
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
        FL_LEFT     = []
        LEFT        = []
        BACK_LEFT   = []

        for i, d in enumerate(scan.ranges):
            if not math.isfinite(d):
                continue
            if d < scan.range_min or d > scan.range_max:
                continue

            ang = angle_min + i * angle_inc

            if -20 <= ang <= 20:
                FRONT.append(d)
            elif -70 <= ang < -20:
                FR_RIGHT.append(d)
            elif -110 <= ang < -70:
                RIGHT.append(d)
            elif -160 <= ang < -110:
                BACK_RIGHT.append(d)
            elif ang <= -160 or ang >= 160:
                BACK.append(d)
            elif 20 < ang <= 70:
                FL_LEFT.append(d)
            elif 70 < ang <= 110:
                LEFT.append(d)
            elif 110 < ang < 160:
                BACK_LEFT.append(d)

        # Minimal distances
        min_front      = min(FRONT)      if FRONT      else float('inf')
        min_fr_right   = min(FR_RIGHT)   if FR_RIGHT   else float('inf')
        min_right      = min(RIGHT)      if RIGHT      else float('inf')
        min_back_right = min(BACK_RIGHT) if BACK_RIGHT else float('inf')
        min_back       = min(BACK) if BACK else float('inf')
        min_fl_left    = min(FL_LEFT) if FL_LEFT else float('inf')
        min_left       = min(LEFT) if LEFT else float('inf')
        min_back_left  = min(BACK_LEFT) if BACK_LEFT else float('inf')

        twist = Twist()
        action = ""

        # Safety emergency: if any sector is dangerously close, back straight
        all_mins = [min_front, min_fr_right, min_right, min_back_right, min_back, min_fl_left, min_left, min_back_left]
        closest = min(all_mins)
        EMERGENCY_DIST = 0.12
        if closest < EMERGENCY_DIST:
            twist.linear.x = -self.v_lin * 0.5
            twist.linear.y = 0.0
            twist.angular.z = 0.0
            action = f"EMERGENCY BACK (closest {closest:.2f} m)"

        #----------------------------------------------------------
        # RULE 1: FRONT obstacle -> move front-left (holonomic)
        #----------------------------------------------------------
        elif min_front < self.base_distance:
            twist.linear.x = self.v_lin * 0.25
            twist.linear.y = +self.v_side
            twist.angular.z = 0.0
            action = f"FRONT {min_front:.2f} m -> MOVE FRONT-LEFT"

        #----------------------------------------------------------
        # RULE 2: FRONT-RIGHT obstacle -> move front-left
        #----------------------------------------------------------
        elif min_fr_right < self.base_distance:
            twist.linear.x = self.v_lin * 0.25
            twist.linear.y = +self.v_side
            twist.angular.z = 0.0
            action = f"FRONT-RIGHT {min_fr_right:.2f} m -> MOVE FRONT-LEFT"

        #----------------------------------------------------------
        # RULE 2b: FRONT-LEFT obstacle -> move front-right
        #----------------------------------------------------------
        elif min_fl_left < self.base_distance:
            twist.linear.x = self.v_lin * 0.25
            twist.linear.y = -self.v_side
            twist.angular.z = 0.0
            action = f"FRONT-LEFT {min_fl_left:.2f} m -> MOVE FRONT-RIGHT"

        #----------------------------------------------------------
        # RULE 3: RIGHT visible -> move forward and maintain parallel (no angular)
        #----------------------------------------------------------
        elif math.isfinite(min_right):
            # lateral correction vy = -kp * error (error = measured - desired)
            error = min_right - self.base_distance
            vy = -self.side_kp * error
            # clamp lateral speed
            vy = max(-self.v_side, min(self.v_side, vy))
            twist.linear.x = self.v_lin
            twist.linear.y = vy
            twist.angular.z = 0.0
            action = (
                f"RIGHT {min_right:.2f} m -> FORWARD (vy={vy:.2f}) maintain parallel"
            )

        #----------------------------------------------------------
        # RULE 3b: LEFT visible -> wall-following (mirror of RIGHT)
        # Improve behavior: when too close to left, reduce forward and strafe right
        # to avoid collisions; when inside tolerance go straight; when far, gently
        # move left to re-acquire the wall. This avoids the diagonal-right bounce
        # when a left wall ends.
        #----------------------------------------------------------
        elif math.isfinite(min_left):
            error = min_left - self.base_distance
            # Too close to left -> back off to the right while slowing forward
            if error < -self.tol:
                twist.linear.x = self.v_lin * 0.3
                twist.linear.y = -self.v_side
                twist.angular.z = 0.0
                action = (
                    f"LEFT too CLOSE {min_left:.2f} m -> slow forward + STRAFE RIGHT"
                )
            # Within tolerance -> go straight and keep orientation
            elif abs(error) <= self.tol:
                twist.linear.x = self.v_lin
                twist.linear.y = 0.0
                twist.angular.z = 0.0
                action = (
                    f"LEFT ~OK ({min_left:.2f} m) -> STRAIGHT"
                )
            # Too far from left -> gently move left to re-acquire wall
            else:
                twist.linear.x = self.v_lin * 0.6
                vy = +min(self.v_side, self.side_kp * error)
                twist.linear.y = vy
                twist.angular.z = 0.0
                action = (
                    f"LEFT too FAR {min_left:.2f} m -> forward + STRAFE LEFT (vy={vy:.2f})"
                )

        #----------------------------------------------------------
        # RULE 4: BACK-RIGHT -> move front-right
        #----------------------------------------------------------
        elif math.isfinite(min_back_right) and (
            not math.isfinite(min_right) or min_back_right <= min_right
        ):
            twist.linear.x = self.v_lin * 0.25
            twist.linear.y = -self.v_side
            twist.angular.z = 0.0
            action = f"BACK-RIGHT {min_back_right:.2f} m -> MOVE FRONT-RIGHT"

        #----------------------------------------------------------
        # RULE 4b: BACK-LEFT -> move front-left
        #----------------------------------------------------------
        elif math.isfinite(min_back_left) and (
            not math.isfinite(min_left) or min_back_left <= min_left
        ):
            twist.linear.x = self.v_lin * 0.25
            twist.linear.y = +self.v_side
            twist.angular.z = 0.0
            action = f"BACK-LEFT {min_back_left:.2f} m -> MOVE FRONT-LEFT"

        #----------------------------------------------------------
        # RULE 5: BACK -> strafe right
        #----------------------------------------------------------
        elif math.isfinite(min_back):
            twist.linear.x = 0.0
            twist.linear.y = -self.v_side
            twist.angular.z = 0.0
            action = f"BACK {min_back:.2f} m -> MOVE RIGHT"

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