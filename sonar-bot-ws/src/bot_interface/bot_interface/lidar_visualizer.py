import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import math

class LidarViz(Node):
    def __init__(self):
        super().__init__('lidar_visualizer')
        self.sub = self.create_subscription(
            Float32MultiArray, 'bot_status_topic', self.callback, 10)
        self.pub = self.create_publisher(MarkerArray, 'lidar_scan_marker', 10)

    def callback(self, msg):
        if len(msg.data) < 6:
            return

        x, y, theta_deg, lidar_angle_deg, distance_cm, _ = msg.data
        theta_rad = math.radians(theta_deg)
        lidar_angle_rad = math.radians(lidar_angle_deg)
        dist_m = distance_cm / 100.0

        # global angle = robot orientation + lidar servo angle
        global_angle = theta_rad + lidar_angle_rad
        pt_x = x + dist_m * math.cos(global_angle)
        pt_y = y + dist_m * math.sin(global_angle)

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "lidar_points"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.03
        marker.scale.y = 0.03
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        from geometry_msgs.msg import Point
        marker.points.append(Point(x=pt_x, y=pt_y, z=0.0))

        self.pub.publish(MarkerArray(markers=[marker]))

def main():
    rclpy.init()
    node = LidarViz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\033[91m[INFO] lidar_visualizer interrupted by user\033[0m")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
