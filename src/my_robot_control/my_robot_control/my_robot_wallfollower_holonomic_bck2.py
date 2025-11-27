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
        self.declare_parameter('distance_limit', 0.5)      # distància desitjada a la paret dreta
        self.declare_parameter('forward_speed', 0.20)
        self.declare_parameter('turn_speed', 0.40)
        self.declare_parameter('side_speed', 0.18)
        self.declare_parameter('side_kp', 1.0)             # CHANGE: una mica més agressiu
        self.declare_parameter('time_to_stop', 60.0)
        self.declare_parameter('tolerance', 0.05)
        self.declare_parameter('front_min', 0.25)          # CHANGE: llindar frontal

        self.base_distance = float(self.get_parameter('distance_limit').value)
        self.v_lin = float(self.get_parameter('forward_speed').value)
        self.v_ang = float(self.get_parameter('turn_speed').value)
        self.v_side = float(self.get_parameter('side_speed').value)
        self.side_kp = float(self.get_parameter('side_kp').value)
        self.time_to_stop = float(self.get_parameter('time_to_stop').value)
        self.tol = float(self.get_parameter('tolerance').value)
        self.front_min = float(self.get_parameter('front_min').value)

        # Last commanded twist
        self.cmd = Twist()

        # ROS 2 entities
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, qos_profile_sensor_data
        )
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timers
        self.info_timer = self.create_timer(1.0, self.log_info)
        self.stop_timer = self.create_timer(0.05, self.stop_watchdog)
        self.cmd_timer = self.create_timer(0.1, self.cmd_publish_timer_cb)

        self._state_action = "Idle"
        self._last_action_logged = None
        self._shutting_down = False

        self.start_time_s = self.get_clock().now().nanoseconds * 1e-9

        self.get_logger().info(
            "WallFollower RIGHT-hand rule, holonomic."
        )

    #--------------------------------------------------------------------
    def stop_watchdog(self):
        if self._shutting_down:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.start_time_s >= self.time_to_stop:
            self.get_logger().info("Stopping due to timeout.")
            self.stop()

    #--------------------------------------------------------------------
    def stop(self):
        self._shutting_down = True
        self.cmd = Twist()
        try:
            self.publisher.publish(self.cmd)
        except Exception:
            pass
        for t in [self.info_timer, self.stop_timer, self.cmd_timer]:
            try:
                t.cancel()
            except Exception:
                pass

    #--------------------------------------------------------------------
    def cmd_publish_timer_cb(self):
        if self._shutting_down:
            return
        try:
            self.publisher.publish(self.cmd)
        except Exception:
            pass

    #--------------------------------------------------------------------
    def laser_callback(self, scan):
        if self._shutting_down:
            return

        angle_min = math.degrees(scan.angle_min)
        angle_inc = math.degrees(scan.angle_increment)

        FRONT       = []
        FR_RIGHT    = []
        RIGHT       = []
        BACK_RIGHT  = []
        BACK        = []
        FR_LEFT     = []
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
                FR_LEFT.append(d)
            elif 70 < ang <= 110:
                LEFT.append(d)
            elif 110 < ang <= 160:
                BACK_LEFT.append(d)

        # Minimal distances
        min_front      = min(FRONT)      if FRONT      else float('inf')
        min_fr_right   = min(FR_RIGHT)   if FR_RIGHT   else float('inf')
        min_right      = min(RIGHT)      if RIGHT      else float('inf')
        min_back_right = min(BACK_RIGHT) if BACK_RIGHT else float('inf')
        min_back       = min(BACK)       if BACK       else float('inf')
        min_fr_left    = min(FR_LEFT)    if FR_LEFT    else float('inf')
        min_left       = min(LEFT)       if LEFT       else float('inf')
        min_back_left  = min(BACK_LEFT)  if BACK_LEFT  else float('inf')

        twist = Twist()
        action = ""

        #----------------------------------------------------------
        # 0) FRONT molt a prop -> girar in situ
        #----------------------------------------------------------  # CHANGE
        if min_front < self.front_min:
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.angular.z = self.v_ang
            action = f"FRONT VERY CLOSE {min_front:.2f} -> TURN LEFT"

        #----------------------------------------------------------
        # 1) FRONT obstacle, però no hi ha bona paret dreta
        #    (si hi ha paret dreta bona, preferim seguir-la)
        #----------------------------------------------------------  # CHANGE condició
        elif self.front_min <= min_front < self.base_distance and \
             (not math.isfinite(min_right) or min_right > self.base_distance + self.tol) and \
             min_front < min_left:
            twist.linear.x = self.v_lin * 0.25
            twist.linear.y = +0.5 * self.v_side      # menys lateral
            twist.angular.z = 0.0
            action = f"FRONT {min_front:.2f} -> MOVE FRONT-LEFT"

        #----------------------------------------------------------
        # 2) FRONT-RIGHT obstacle (igual criteri)
        #----------------------------------------------------------  # CHANGE condició
        elif min_fr_right < self.base_distance and \
             (not math.isfinite(min_right) or min_right > self.base_distance + self.tol):
            twist.linear.x = self.v_lin * 0.25
            twist.linear.y = +0.5 * self.v_side
            twist.angular.z = 0.0
            action = f"FRONT-RIGHT {min_fr_right:.2f} -> MOVE FRONT-LEFT"

        #-------------------------------------------------------------------------
        # 3) RIGHT visible -> MODE SEGUIR PARET DRETA
        #-------------------------------------------------------------------------
        elif math.isfinite(min_right):
            error = min_right - self.base_distance
            vy = -self.side_kp * error
            vy = max(-self.v_side, min(self.v_side, vy))
            twist.linear.x = 0.15                     # CHANGE: una mica més lent
            twist.linear.y = vy
            twist.angular.z = 0.0
            action = f"RIGHT {min_right:.2f} -> FOLLOW RIGHT WALL (vy={vy:.2f})"

        #----------------------------------------------------------
        # 4) BACK-RIGHT -> acostar-se a la paret dreta
        #----------------------------------------------------------
        elif math.isfinite(min_back_right) and (
            not math.isfinite(min_right) or min_back_right <= min_right
        ):
            twist.linear.x = self.v_lin * 0.25
            twist.linear.y = -self.v_side
            twist.angular.z = 0.0
            action = f"BACK-RIGHT {min_back_right:.2f} -> MOVE FRONT-RIGHT"

        #----------------------------------------------------------
        # 5) BACK -> desplaçar-se a la dreta
        #----------------------------------------------------------
        elif math.isfinite(min_back):
            twist.linear.x = 0.0
            twist.linear.y = -self.v_side
            twist.angular.z = 0.0
            action = f"BACK {min_back:.2f} -> MOVE RIGHT"

        #----------------------------------------------------------
        # 6) FRONT-LEFT obstacle -> escapa cap a la dreta
        #----------------------------------------------------------
        elif min_fr_left < self.base_distance:
            twist.linear.x = self.v_lin * 0.25
            twist.linear.y = -self.v_side
            twist.angular.z = 0.0
            action = f"FRONT-LEFT {min_fr_left:.2f} -> MOVE FRONT-RIGHT"

        #------------------------------------------------------------------------
        # 7) LEFT visible -> paret a l'esquerra (no la volem), fem una mica dreta
        #------------------------------------------------------------------------  # CHANGE
        elif math.isfinite(min_left):
            twist.linear.x = 0.15
            twist.linear.y = -0.1
            twist.angular.z = 0.0
            action = f"LEFT {min_left:.2f} -> MOVE SLIGHTLY RIGHT"

        #----------------------------------------------------------
        # 8) BACK-LEFT -> tirar cap endavant perquè aparegui paret dreta
        #----------------------------------------------------------
        elif math.isfinite(min_back_left) and (
            not math.isfinite(min_left) or min_back_left <= min_left
        ):
            twist.linear.x = self.v_lin * 0.25
            twist.linear.y = +self.v_side
            twist.angular.z = 0.0
            action = f"BACK-LEFT {min_back_left:.2f} -> MOVE FRONT-LEFT"

        #----------------------------------------------------------
        # 9) CAP MUR CLAR -> MODE BUSCAR PARET DRETA
        #----------------------------------------------------------  # CHANGE
        else:
            twist.linear.x = 0.15
            twist.linear.y = -0.10      # lleugerament cap a la dreta
            twist.angular.z = 0.0
            action = "SEARCHING RIGHT WALL"

        # Update last commanded twist
        self.cmd = twist

        if action != self._last_action_logged:
            self.get_logger().info(action)
            self._last_action_logged = action

        self._state_action = action

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
