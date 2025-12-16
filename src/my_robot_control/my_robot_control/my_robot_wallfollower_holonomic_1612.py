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
        self.declare_parameter('distance_limit', 0.7)    # desired distance to right wall
        self.declare_parameter('forward_speed', 0.20)    # linear speed
        self.declare_parameter('turn_speed', 0.40)       # angular speed
        self.declare_parameter('time_to_stop', 30.0)     # auto-stop
        self.declare_parameter('tolerance', 0.1)        # band around base_distance (RIGHT)

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
        self.info_timer = self.create_timer(2.0, self.log_info)
        self.stop_timer = self.create_timer(0.01, self.stop_watchdog)

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
        if self._shutting_down:
            return

        angle_min = math.degrees(scan.angle_min)
        angle_inc = math.degrees(scan.angle_increment)

        # Sector arrays
        sectors = {
            "FRONT": [],
            "FR_RIGHT": [],
            "RIGHT": [],
            "BACK_RIGHT": [],
            "FRONT_LEFT": [],
            "LEFT": [],
            "BACK_LEFT": [],
            "BACK": []
        }

        # Sort lidar measurements into sectors
        for i, d in enumerate(scan.ranges):
            if not math.isfinite(d):
                continue
            if d < scan.range_min or d > scan.range_max:
                continue

            ang = angle_min + i * angle_inc

            if -20 <= ang <= 20:
                sectors["FRONT"].append(d)
            elif -70 <= ang < -20:
                sectors["FR_RIGHT"].append(d)
            elif -110 <= ang < -70:
                sectors["RIGHT"].append(d)
            elif -160 <= ang < -110:
                sectors["BACK_RIGHT"].append(d)
            elif 20 < ang <= 70:
                sectors["FRONT_LEFT"].append(d)
            elif 70 < ang <= 110:
                sectors["LEFT"].append(d)
            elif 110 < ang <= 160:
                sectors["BACK_LEFT"].append(d)
            elif ang >= 160 or ang <= -160:
                sectors["BACK"].append(d)

        # Compute minimum per sector
        min_dist = {}
        for name, values in sectors.items():
            min_dist[name] = min(values) if values else float('inf')

        # Find absolute minimum and sector where it occurs
        closest_sector = min(min_dist, key=min_dist.get)
        closest_value = min_dist[closest_sector]

        twist = Twist()
        action = ""

        # If nothing is close --> stop
        if closest_value == float('inf'):
            self.cmd = Twist()
            self._state_action = "CLEAR --> STOP"
            return

        # ----------- REACT ONLY WHEN BELOW LIMIT -------------
        if closest_value >= self.base_distance:
            # Nothing too close --> go forward
            twist.linear.x = self.v_lin
            twist.linear.y = 0.0
            twist.angular.z = 0.0
            action = f"SAFE ({closest_value:.2f}) --> FORWARD"
        else:
            # ----------- MIN DISTANCE REACHED --> MOVE HOLONOMICALLY ----------
            if closest_sector == "FRONT":
                twist.linear.x = 0.0
                twist.linear.y = +self.v_lin
                action = f"FRONT {closest_value:.2f} --> MOVE LEFT"

            elif closest_sector == "FR_RIGHT":
                twist.linear.x = +self.v_lin
                twist.linear.y = +self.v_lin
                action = f"FRONT-RIGHT {closest_value:.2f} --> MOVE FRONT-LEFT"

            elif closest_sector == "RIGHT":
                twist.linear.x = +self.v_lin
                twist.linear.y = 0.0
                action = f"RIGHT {closest_value:.2f} --> MOVE FORWARD"

            elif closest_sector == "BACK_RIGHT":
                twist.linear.x = +self.v_lin
                twist.linear.y = -self.v_lin
                action = f"BACK-RIGHT {closest_value:.2f} --> MOVE FRONT-RIGHT"

            elif closest_sector == "FRONT_LEFT":
                twist.linear.x = -self.v_lin
                twist.linear.y = +self.v_lin
                action = f"FRONT-LEFT {closest_value:.2f} --> MOVE BACK-LEFT"

            elif closest_sector == "LEFT":
                twist.linear.x = -self.v_lin
                twist.linear.y = 0.0
                action = f"LEFT {closest_value:.2f} --> MOVE BACKWARDS"

            elif closest_sector == "BACK_LEFT":
                twist.linear.x = -self.v_lin
                twist.linear.y = -self.v_lin
                action = f"BACK-LEFT {closest_value:.2f} --> MOVE BACK-RIGHT"

            elif closest_sector == "BACK":
                twist.linear.x = 0.0
                twist.linear.y = -self.v_lin
                action = f"BACK {closest_value:.2f} --> MOVE RIGHT"

        # Store and publish
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