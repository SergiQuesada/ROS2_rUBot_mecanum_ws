import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math


class RobotSelfControlHolonomic(Node):

    def __init__(self):
        super().__init__('robot_selfcontrol_holonomic')

        # PARAMETERS
        self.declare_parameter('distance_limit', 0.4)
        self.declare_parameter('speed_factor', 1.0)
        self.declare_parameter('forward_speed', 0.2)
        self.declare_parameter('rotation_speed', 0.4)
        self.declare_parameter('time_to_stop', 10.0)

        self.d_limit = self.get_parameter('distance_limit').value
        self.k = self.get_parameter('speed_factor').value
        self.vf = self.get_parameter('forward_speed').value
        self.wz = self.get_parameter('rotation_speed').value
        self.time_to_stop = self.get_parameter('time_to_stop').value

        # message to publish
        self.cmd = Twist()
        self.cmd.linear.x = self.vf
        self.cmd.linear.y = 0.0
        self.cmd.angular.z = 0.0

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)

        self.start_time = self.get_clock().now().nanoseconds * 1e-9
        self.last_info = self.start_time
        self.last_speed_log = self.start_time
        self.stopping = False


    # ----------------------------------------------------------
    # TIMER → publish cmd_vel
    # ----------------------------------------------------------
    def timer_callback(self):
        if self.stopping:
            return
        
        self.pub.publish(self.cmd)

        now = self.get_clock().now().nanoseconds * 1e-9

        # Log speed every second
        if now - self.last_speed_log >= 1.0:
            self.get_logger().info(
                f"Vx: {self.cmd.linear.x:.2f}, Vy: {self.cmd.linear.y:.2f}, w:{self.cmd.angular.z:.2f}"
            )
            self.last_speed_log = now

        # Stop after timeout
        if now - self.start_time >= self.time_to_stop:
            self.stop_robot()
            self.get_logger().info("Robot stopped")
            rclpy.try_shutdown()


    # ----------------------------------------------------------
    # LIDAR CALLBACK
    # ----------------------------------------------------------
    def laser_callback(self, scan):
        if self.stopping:
            return

        angle_min = scan.angle_min
        angle_inc = scan.angle_increment

        valid_points = []

        for i, dist in enumerate(scan.ranges):
            if not math.isfinite(dist) or dist <= 0.0:
                continue
            if dist < scan.range_min or dist > scan.range_max:
                continue

            angle = angle_min + i * angle_inc
            angle_deg = math.degrees(angle)

            # Wrap angle to [-180,180]
            if angle_deg > 180:
                angle_deg -= 360

            # Filter full FOV
            if -150 <= angle_deg <= 150:
                valid_points.append((dist, angle_deg))

        if not valid_points:
            return

        # CLosest point
        closest_dist, closest_angle = min(valid_points)

        # Zones
        if -45 <= closest_angle <= 45:
            zone = "FRONT"
        elif 45 < closest_angle <= 110:
            zone = "LEFT"
        elif -110 <= closest_angle < -45:
            zone = "RIGHT"
        else:
            zone = "OTHER"

        # Log detection
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_info >= 1.0:
            self.get_logger().info(
                f"[DETECTION] Dist: {closest_dist:.2f}m | Angle: {closest_angle:.0f}° | Zone: {zone}"
            )
            self.last_info = now

        # ----------------------------------------------------------
        # ANTI-ENCALLAMIENTO: DETECTAR EMPATE LADO IZQ/DER
        # ----------------------------------------------------------
        left_hits = [p for p in valid_points if 45 < p[1] <= 110]
        right_hits = [p for p in valid_points if -110 <= p[1] < -45]

        if left_hits and right_hits:
            left_min = min(left_hits)[0]
            right_min = min(right_hits)[0]

            # Si son prácticamente iguales (empate → pasillo estrecho)
            if abs(left_min - right_min) < 0.05:  # 5 cm
                # Seleccionamos el que está más hacia delante (menor |angulo|)
                worst_left = min(left_hits, key=lambda p: abs(p[1]))
                worst_right = min(right_hits, key=lambda p: abs(p[1]))

                # Si el izquierdo está más hacia adelante → es más peligroso
                if abs(worst_left[1]) < abs(worst_right[1]):
                    zone = "LEFT"   # mover hacia derecha
                else:
                    zone = "RIGHT"  # mover hacia izquierda


        # ----------------------------------------------------------
        # REACTION BEHAVIOR (HOLONOMIC)
        # ----------------------------------------------------------
        if closest_dist < self.d_limit:

            if zone == "FRONT":
                # Retrocede en X
                self.cmd.linear.x = -self.vf
                self.cmd.linear.y = 0.0
                self.cmd.angular.z = 0.0

            elif zone == "LEFT":
                # Obstáculo a la izquierda → moverse a la derecha (Vy -)
                self.cmd.linear.x = 0.0
                self.cmd.linear.y = -self.vf
                self.cmd.angular.z = 0.0

            elif zone == "RIGHT":
                # Obstáculo a la derecha → moverse a la izquierda (Vy +)
                self.cmd.linear.x = 0.0
                self.cmd.linear.y = self.vf
                self.cmd.angular.z = 0.0

            else:
                # Zona atrás o rara → avanzar normal
                self.cmd.linear.x = self.vf
                self.cmd.linear.y = 0.0
                self.cmd.angular.z = 0.0

        else:
            # PATH CLEAR → avanzar hacia adelante
            self.cmd.linear.x = self.vf
            self.cmd.linear.y = 0.0
            self.cmd.angular.z = 0.0


    # ----------------------------------------------------------
    def stop_robot(self):
        self.stopping = True
        stop = Twist()
        self.pub.publish(stop)


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
