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
        self.declare_parameter('distance_limit', 0.5)    # distancia deseada a muro
        self.declare_parameter('forward_speed', 0.20)    # velocidad en x
        self.declare_parameter('turn_speed', 0.40)       # vel. angular z
        self.declare_parameter('side_speed', 0.18)       # velocidad en y
        self.declare_parameter('time_to_stop', 120.0)    # auto-stop
        self.declare_parameter('tolerance', 0.05)

        self.base_distance = float(self.get_parameter('distance_limit').value)
        self.v_lin = float(self.get_parameter('forward_speed').value)
        self.v_ang = float(self.get_parameter('turn_speed').value)
        self.v_side = float(self.get_parameter('side_speed').value)
        self.time_to_stop = float(self.get_parameter('time_to_stop').value)
        self.tol = float(self.get_parameter('tolerance').value)

        # Último comando
        self.cmd = Twist()

        # Subs y pubs
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

        # Máquina de estados
        self.state = "FORWARD_SEARCH"
        self.turn_start_time = None     # para controlar giro 180º con el reloj del nodo
        self.turn_duration = math.pi / self.v_ang  # tiempo para ~180º: ángulo/velocidad [web:16][web:47]

        self.get_logger().info("WallFollower holonómico con giro 180º en esquinas izquierda-atrás.")

    # --------------------------------------------------------------------
    def stop_watchdog(self):
        if self._shutting_down:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.start_time_s >= self.time_to_stop:
            self.get_logger().info("Stopping due to timeout.")
            self.stop()

    # --------------------------------------------------------------------
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

    # --------------------------------------------------------------------
    def cmd_publish_timer_cb(self):
        if self._shutting_down:
            return
        try:
            self.publisher.publish(self.cmd)
        except Exception:
            pass

    # --------------------------------------------------------------------
    def laser_callback(self, scan: LaserScan):
        if self._shutting_down:
            return

        angle_min = math.degrees(scan.angle_min)
        angle_inc = math.degrees(scan.angle_increment)

        FRONT = []
        FRONT_LEFT = []
        LEFT = []
        BACK = []

        for i, d in enumerate(scan.ranges):
            if not math.isfinite(d):
                continue
            if d < scan.range_min or d > scan.range_max:
                continue

            ang = angle_min + i * angle_inc

            # 0º = delante, + a la izquierda, - a la derecha
            if -20 <= ang <= 20:
                FRONT.append(d)
            elif 20 < ang <= 70:
                FRONT_LEFT.append(d)
            elif 70 < ang <= 110:
                LEFT.append(d)
            elif ang <= -160 or ang >= 160:
                BACK.append(d)

        min_front = min(FRONT) if FRONT else float('inf')
        min_front_left = min(FRONT_LEFT) if FRONT_LEFT else float('inf')
        min_left = min(LEFT) if LEFT else float('inf')
        min_back = min(BACK) if BACK else float('inf')

        twist = Twist()
        action = ""

        now = self.get_clock().now().nanoseconds * 1e-9

        # ----------------- LÓGICA DE ESTADOS -----------------

        if self.state == "FORWARD_SEARCH":
            # Avanza hasta encontrar un muro delante
            if min_front < self.base_distance:
                self.state = "FOLLOW_LEFT"
                twist.linear.x = 0.0
                twist.linear.y = +self.v_side
                action = f"FOUND FRONT WALL {min_front:.2f} -> FOLLOW_LEFT (STRAFE LEFT)"
            else:
                twist.linear.x = self.v_lin
                twist.linear.y = 0.0
                action = f"FORWARD_SEARCH -> MOVE FORWARD (front={min_front:.2f})"

        elif self.state == "FOLLOW_LEFT":
            # If front is cleared, resume moving forward (don't keep strafing left)
            if min_front >= self.base_distance:
                self.state = "FORWARD_SEARCH"
                twist.linear.x = self.v_lin
                twist.linear.y = 0.0
                action = f"FOLLOW_LEFT: front cleared (front={min_front:.2f}) -> MOVE FORWARD"
            # Muro muy cerca a la izquierda o frontal-izquierda
            elif min_left < self.base_distance or min_front_left < self.base_distance:
                if min_back < self.base_distance:
                    # Esquina detrás-izquierda: pasar a girar 180º en el sitio
                    self.state = "TURN_AROUND"
                    self.turn_start_time = now
                    twist.linear.x = 0.0
                    twist.linear.y = 0.0
                    twist.angular.z = +self.v_ang  # giro antihorario [web:16][web:46]
                    action = (
                        f"CORNER BACK+LEFT: LEFT {min_left:.2f}, BACK {min_back:.2f} "
                        f"-> TURN_AROUND (start)"
                    )
                else:
                    # Solo muro izquierda/frontal-izq: retroceder
                    self.state = "BACK_FROM_LEFT"
                    twist.linear.x = -self.v_lin
                    twist.linear.y = 0.0
                    action = (
                        f"LEFT {min_left:.2f} / FRONT_LEFT {min_front_left:.2f} "
                        f"-> BACK_FROM_LEFT (MOVE BACKWARD)"
                    )
            else:
                # Seguir bordeando hacia la izquierda
                twist.linear.x = 0.0
                twist.linear.y = +self.v_side
                action = (
                    f"FOLLOW_LEFT: front={min_front:.2f}, left={min_left:.2f} "
                    f"-> STRAFE LEFT"
                )

        elif self.state == "BACK_FROM_LEFT":
            # Mientras siga habiendo muro a la izquierda/diagonal, retrocede
            if min_left < self.base_distance or min_front_left < self.base_distance:
                if min_back < self.base_distance:
                    # También muro detrás -> gira 180º
                    self.state = "TURN_AROUND"
                    self.turn_start_time = now
                    twist.linear.x = 0.0
                    twist.linear.y = 0.0
                    twist.angular.z = +self.v_ang
                    action = (
                        f"BACK_FROM_LEFT CORNER: LEFT {min_left:.2f}, BACK {min_back:.2f} "
                        f"-> TURN_AROUND (start)"
                    )
                else:
                    twist.linear.x = -self.v_lin
                    twist.linear.y = 0.0
                    action = (
                        f"BACK_FROM_LEFT: still LEFT {min_left:.2f} / FRONT_LEFT {min_front_left:.2f} "
                        f"-> MOVE BACKWARD"
                    )
            else:
                # Ya no hay muro a la izquierda -> volver a seguirlo
                self.state = "FOLLOW_LEFT"
                twist.linear.x = 0.0
                twist.linear.y = +self.v_side
                action = (
                    f"BACK_FROM_LEFT: left cleared (left={min_left:.2f}) "
                    f"-> FOLLOW_LEFT (STRAFE LEFT)"
                )

        elif self.state == "TURN_AROUND":
            # Girar en el sitio durante turn_duration para ~180º
            if self.turn_start_time is None:
                self.turn_start_time = now

            elapsed = now - self.turn_start_time
            if elapsed < self.turn_duration:
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                twist.angular.z = +self.v_ang
                action = f"TURN_AROUND: rotating (elapsed={elapsed:.2f}/{self.turn_duration:.2f}s)"
            else:
                # Fin del giro -> iniciar de nuevo búsqueda hacia la izquierda
                self.state = "FOLLOW_LEFT"
                self.turn_start_time = None
                twist.linear.x = 0.0
                twist.linear.y = +self.v_side
                twist.angular.z = 0.0
                action = "TURN_AROUND done -> FOLLOW_LEFT (STRAFE LEFT)"

        else:
            self.state = "FORWARD_SEARCH"
            twist.linear.x = self.v_lin
            twist.linear.y = 0.0
            action = f"UNKNOWN STATE -> RESET TO FORWARD_SEARCH (front={min_front:.2f})"

        self.cmd = twist

        if action != self._last_action_logged:
            self.get_logger().info(f"[{self.state}] {action}")
            self._last_action_logged = action

        self._state_action = f"{self.state}: {action}"

    # --------------------------------------------------------------------
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
