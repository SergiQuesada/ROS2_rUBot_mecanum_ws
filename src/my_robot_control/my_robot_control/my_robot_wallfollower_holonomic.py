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
        self.declare_parameter('distance_limit', 0.5)    # desired distance to wall
        self.declare_parameter('forward_speed', 0.20)    # linear speed (x)
        self.declare_parameter('turn_speed', 0.40)       # angular speed
        self.declare_parameter('side_speed', 0.18)       # lateral speed (y)
        self.declare_parameter('side_kp', 0.8)           # kept for compatibility
        self.declare_parameter('time_to_stop', 30.0)     # auto-stop
        self.declare_parameter('tolerance', 0.05)

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
            "WallFollower holonómico siguiendo muros."
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

        # Try a final publish
        try:
            self.publisher.publish(self.cmd)
        except Exception:
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
        FRONT_LEFT  = []
        LEFT        = []

        for i, d in enumerate(scan.ranges):
            if not math.isfinite(d):
                continue
            if d < scan.range_min or d > scan.range_max:
                continue

            ang = angle_min + i * angle_inc

            # 0º delante, + a la izquierda, - a la derecha
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
                FRONT_LEFT.append(d)
            elif 70 < ang <= 110:
                LEFT.append(d)

        # Minimal distances
        min_front       = min(FRONT)      if FRONT      else float('inf')
        min_fr_right    = min(FR_RIGHT)   if FR_RIGHT   else float('inf')
        min_right       = min(RIGHT)      if RIGHT      else float('inf')
        min_back_right  = min(BACK_RIGHT) if BACK_RIGHT else float('inf')
        min_back        = min(BACK)       if BACK       else float('inf')
        min_front_left  = min(FRONT_LEFT) if FRONT_LEFT else float('inf')
        min_left        = min(LEFT)       if LEFT       else float('inf')

        twist = Twist()
        action = ""

        # REGLA A: muro delante -> ir a la izquierda holonómicamente
        if min_front < self.base_distance:
            twist.linear.x = 0.0
            twist.linear.y = +self.v_side      # +y = izquierda
            twist.angular.z = 0.0
            action = f"FRONT {min_front:.2f} m -> STRAFE LEFT"

        # REGLA B: muro en front-left o left -> retroceder mientras haya muro a la izquierda
        elif min_front_left < self.base_distance or min_left < self.base_distance:
            if min_back < self.base_distance:
                # REGLA C: muro izquierda y detrás -> ir a la derecha (única salida)
                twist.linear.x = 0.0
                twist.linear.y = -self.v_side   # -y = derecha
                twist.angular.z = 0.0
                action = (
                    f"LEFT {min_left:.2f} / FRONT-LEFT {min_front_left:.2f} m "
                    f"y BACK {min_back:.2f} m -> STRAFE RIGHT"
                )
            else:
                # Retroceder manteniendo muro a la izquierda
                twist.linear.x = -self.v_lin
                twist.linear.y = 0.0
                twist.angular.z = 0.0
                action = (
                    f"LEFT {min_left:.2f} / FRONT-LEFT {min_front_left:.2f} m "
                    f"-> MOVE BACKWARD"
                )

        # REGLA D: sin muros cercanos relevantes -> deriva suave hacia la izquierda
        else:
            twist.linear.x = 0.0
            twist.linear.y = +self.v_side * 0.5
            twist.angular.z = 0.0
            action = "NO WALL NEARBY -> DRIFT LEFT"

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
