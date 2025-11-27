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

        # Minimal state for corner handling (see user request)
        self.front_blocked = False
        self.backing = False
        self.left_blocked = False

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
        FRONT_LEFT  = []
        
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
            elif 110 <= ang < 160:
                BACK_LEFT.append(d)
            elif 70 <= ang < 110:
                LEFT.append(d)
            elif 20 < ang < 70:
                FRONT_LEFT.append(d)

        # Minimal distances
        min_front      = min(FRONT)      if FRONT      else float('inf')
        min_fr_right   = min(FR_RIGHT)   if FR_RIGHT   else float('inf')
        min_right      = min(RIGHT)      if RIGHT      else float('inf')
        min_back_right = min(BACK_RIGHT) if BACK_RIGHT else float('inf')
        min_back       = min(BACK) if BACK else float('inf')
        min_back_left  = min(BACK_LEFT) if BACK_LEFT else float('inf')
        min_left       = min(LEFT) if LEFT else float('inf')
        min_front_left = min(FRONT_LEFT) if FRONT_LEFT else float('inf')

        twist = Twist()
        action = ""

        # ------------------ Corner / front handling ------------------
        # If something is in front, start a corner-handling state:
        #  - when front detected -> strafe left and remember front_blocked
        #  - while front_blocked: if left clears -> advance (forward-left)
        #    otherwise back up until left clears; if back is blocked -> move right
        if min_front < self.base_distance:
            # immediate front obstacle: move left and mark blocked
            self.front_blocked = True
            self.backing = False
            twist.linear.x = self.v_lin * 0.25
            twist.linear.y = +self.v_side
            twist.angular.z = 0.0
            action = f"FRONT {min_front:.2f} m -> STRAFE LEFT (start corner handling)"

        elif self.front_blocked:
            # we previously hit something in front; try to resolve
            # if left side is free, try to go forward-left and clear the flag
            if min_left > self.base_distance + self.tol:
                twist.linear.x = self.v_lin * 0.5
                twist.linear.y = +self.v_side
                twist.angular.z = 0.0
                action = (
                    f"FRONT_BLOCKED: left cleared (left={min_left:.2f}) -> FORWARD-LEFT and resume"
                )
                self.front_blocked = False
                self.backing = False

            else:
                # left still blocked / corner not cleared: if still blocked in front, back up
                if min_front < self.base_distance:
                    twist.linear.x = -self.v_lin * 0.25
                    twist.linear.y = 0.0
                    twist.angular.z = 0.0
                    self.backing = True
                    action = f"FRONT_BLOCKED: still front {min_front:.2f} -> BACKING"

                # if we are backing and there's an obstacle behind, try to move right to escape
                elif self.backing and min_back < self.base_distance:
                    twist.linear.x = 0.0
                    twist.linear.y = -self.v_side
                    twist.angular.z = 0.0
                    action = (
                        f"FRONT_BLOCKED: back blocked {min_back:.2f} -> MOVE RIGHT to escape"
                    )

                else:
                    # keep trying to slide left slowly while monitoring
                    twist.linear.x = self.v_lin * 0.15
                    twist.linear.y = +self.v_side
                    twist.angular.z = 0.0
                    action = "FRONT_BLOCKED: sliding LEFT while waiting for clearance"

        # ------------------ LEFT collision handling ------------------
        # If the robot detects the wall too close on the left side, back
        # until front-left is clear and then strafe left to resume following.
        elif min_left < self.base_distance and not self.left_blocked:
            self.left_blocked = True
            self.backing = True
            twist.linear.x = -self.v_lin * 0.25
            twist.linear.y = 0.0
            twist.angular.z = 0.0
            action = f"LEFT too close ({min_left:.2f}) -> BACKING to clear front-left"

        elif self.left_blocked:
            # if front-left clears, strafe left and resume
            if min_front_left > self.base_distance + self.tol:
                twist.linear.x = self.v_lin * 0.4
                twist.linear.y = +self.v_side
                twist.angular.z = 0.0
                action = (
                    f"LEFT_BLOCKED cleared (front-left={min_front_left:.2f}) -> STRAFE LEFT and resume"
                )
                self.left_blocked = False
                self.backing = False

            else:
                # keep backing unless back is blocked, in which case move right
                if self.backing:
                    if min_back < self.base_distance:
                        twist.linear.x = 0.0
                        twist.linear.y = -self.v_side
                        twist.angular.z = 0.0
                        action = f"LEFT_BLOCKED: back blocked {min_back:.2f} -> MOVE RIGHT to escape"
                    else:
                        twist.linear.x = -self.v_lin * 0.25
                        twist.linear.y = 0.0
                        twist.angular.z = 0.0
                        action = f"LEFT_BLOCKED: backing (front-left={min_front_left:.2f})"
                else:
                    twist.linear.x = -self.v_lin * 0.15
                    twist.linear.y = 0.0
                    twist.angular.z = 0.0
                    action = "LEFT_BLOCKED: small backoff"

        #----------------------------------------------------------
        # RULE 2: FRONT-RIGHT obstacle -> move front-left
        #----------------------------------------------------------
        elif min_fr_right < self.base_distance:
            twist.linear.x = self.v_lin * 0.25
            twist.linear.y = +self.v_side
            twist.angular.z = 0.0
            action = f"FRONT-RIGHT {min_fr_right:.2f} m -> MOVE FRONT-LEFT"

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