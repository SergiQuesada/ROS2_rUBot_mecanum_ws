import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class WallFollowerHolonomic(Node):
    def __init__(self):
        super().__init__('wall_follower_holonomic_node')

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
            "WallFollower (Holonomic) - using lateral strafing for obstacle avoidance."
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
            elif ang > 160 or ang < -160:
                BACK.append(d)

        # Minimal distances
        mins = {
            'front': min(FRONT)      if FRONT      else float('inf'),
            'fr_right': min(FR_RIGHT) if FR_RIGHT  else float('inf'),
            'right': min(RIGHT)      if RIGHT      else float('inf'),
            'back_right': min(BACK_RIGHT) if BACK_RIGHT else float('inf'),
            'back': min(BACK)        if BACK       else float('inf'),
        }

        min_front      = mins['front']
        min_fr_right   = mins['fr_right']
        min_right      = mins['right']
        min_back_right = mins['back_right']
        min_back       = mins['back']

        twist = Twist()
        action = ""

        # Safety thresholds
        emergency_dist = 0.12                        # stop immediately if anything closer than this
        close_thresh = min(max(0.18, self.base_distance * 0.6), 0.35)  # "very close" threshold
        lateral_max = max(self.v_lin, self.v_ang * 1.5)  # clamp lateral speed

        def clamp(v, a, b):
            return max(a, min(b, v))

        # EMERGENCY STOP: anything extremely close -> stop immediately
        if min(min_front, min_fr_right, min_right, min_back_right, min_back) < emergency_dist:
            twist = Twist()  # all zeros
            action = f"EMERGENCY STOP (obstacle < {emergency_dist:.2f} m)"
        #----------------------------------------------------------
        # RULE 1: FRONT obstacle
        #   - if very close: stop forward and strafe left until cleared
        #   - if moderately close: slow forward + strafe left
        #----------------------------------------------------------
        elif min_front < close_thresh:
            twist.linear.x = 0.0
            twist.linear.y = clamp(self.v_ang * 1.8, -lateral_max, lateral_max)  # strong left strafe
            twist.angular.z = 0.0
            action = f"FRONT very CLOSE {min_front:.2f} m → STOP FORWARD + STRAFE LEFT"
        elif min_front < self.base_distance:
            twist.linear.x = self.v_lin * 0.4
            twist.linear.y = clamp(self.v_ang * 1.2, -lateral_max, lateral_max)  # gentle left strafe
            twist.angular.z = 0.0
            action = f"FRONT {min_front:.2f} m → slow FORWARD + STRAFE LEFT"

        #----------------------------------------------------------
        # RULE 2: FRONT-RIGHT obstacle
        #   - if very close: stop forward and strafe left
        #   - else: forward + left strafe but slower
        #----------------------------------------------------------
        elif min_fr_right < close_thresh:
            twist.linear.x = 0.0
            twist.linear.y = clamp(self.v_ang * 1.5, -lateral_max, lateral_max)
            twist.angular.z = 0.0
            action = f"FRONT-RIGHT VERY CLOSE {min_fr_right:.2f} m → STOP FORWARD + STRAFE LEFT"
        elif min_fr_right < self.base_distance:
            twist.linear.x = self.v_lin * 0.6
            twist.linear.y = clamp(self.v_ang * 1.0, -lateral_max, lateral_max)
            twist.angular.z = 0.0
            action = f"FRONT-RIGHT {min_fr_right:.2f} m → FORWARD + STRAFE LEFT"

        #----------------------------------------------------------
        # RULE 3: RIGHT visible → move forward and maintain orientation parallel to the wall
        #   - use lateral proportional control, clamp speeds
        #----------------------------------------------------------
        elif math.isfinite(min_right):
            # error > 0 → too far; error < 0 → too close
            error = min_right - self.base_distance

            k_lat = 1.2  # lateral gain
            lateral = -k_lat * error
            lateral = clamp(lateral, -lateral_max, lateral_max)

            twist.linear.x = self.v_lin
            twist.linear.y = lateral
            twist.angular.z = 0.0

            if abs(error) <= self.tol:
                action = (
                    f"RIGHT ~OK ({min_right:.2f} m) → FORWARD (maintain)"
                )
            elif error < 0:
                action = (
                    f"RIGHT too CLOSE ({min_right:.2f} m) → FORWARD + strafe LEFT"
                )
            else:
                action = (
                    f"RIGHT too FAR ({min_right:.2f} m) → FORWARD + strafe RIGHT"
                )

        #----------------------------------------------------------
        # RULE 4: BACK-RIGHT -> avoid by moving forward+right (but ensure not moving into front obstacle)
        #----------------------------------------------------------
        elif math.isfinite(min_back_right) and (
            not math.isfinite(min_right) or min_back_right <= min_right
        ):
            twist.linear.x = self.v_lin * 0.6
            twist.linear.y = clamp(-self.v_ang * 1.0, -lateral_max, lateral_max)  # right strafe
            twist.angular.z = 0.0
            action = f"BACK-RIGHT {min_back_right:.2f} m → FORWARD + STRAFE RIGHT"

        #----------------------------------------------------------
        # RULE 5: BACK obstacle -> move right (strafe) to clear
        #----------------------------------------------------------
        elif math.isfinite(min_back) and min_back < self.base_distance:
            twist.linear.x = 0.0
            twist.linear.y = clamp(-self.v_ang * 1.6, -lateral_max, lateral_max)
            twist.angular.z = 0.0
            action = f"BACK {min_back:.2f} m → STRAFE RIGHT"

        # else: nothing detected, keep zero (robot stops)

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
    node = WallFollowerHolonomic()
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