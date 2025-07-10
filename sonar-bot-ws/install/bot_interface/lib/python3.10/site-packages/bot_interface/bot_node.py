import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
import sys, termios, tty, threading, time

class BotNode(Node):
    def __init__(self):
        super().__init__('bot_interface_node')
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_sub = self.create_subscription(
            Float32MultiArray, 'bot_status_topic', self.status_callback, 10)

        self.v = 0.0
        self.w = 0.0
        self.running = True

        # Start threads
        threading.Thread(target=self.keyboard_loop, daemon=True).start()
        self.create_timer(0.1, self.send_cmd)  # Publish every 100ms

    def status_callback(self, msg):
        if len(msg.data) == 6:
            x, y, theta, lidar_angle, distance, voltage = msg.data
            self.get_logger().info(
                f'Pose: ({x:.2f}, {y:.2f}, {theta:.1f}°), Lidar: {lidar_angle:.0f}° = {distance:.1f}cm, Battery: {voltage:.2f}V'
            )

    def send_cmd(self):
        twist = Twist()
        twist.linear.x = self.v
        twist.angular.z = self.w
        self.cmd_pub.publish(twist)

        # Update command line output in-place
        color = (
            "\033[92m" if self.v > 0 else
            "\033[94m" if self.v < 0 else
            "\033[93m" if self.w != 0 else
            "\033[90m"
        )
        status_line = f"{color}[COMMAND] v: {self.v:.2f}  w: {self.w:.2f}      \033[0m"
        print(f"\r{status_line}", end='', flush=True)

    def get_key(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        return ch

    def keyboard_loop(self):
        print("\nControl: W/S = fwd/rev | A/D = turn | Space = stop | Q = quit")
        while self.running:
            key = self.get_key()
            if key == 'w':
                self.v = min(self.v + 0.05, 0.3)
            elif key == 's':
                self.v = max(self.v - 0.05, -0.3)
            elif key == 'a':
                self.w = min(self.w + 0.1, 1.0)
            elif key == 'd':
                self.w = max(self.w - 0.1, -1.0)
            elif key == 'x' or key == ' ':
                self.v = 0.0
                self.w = 0.0
            elif key == 'q':
                self.running = False
                print("\n\033[91m[INFO] Exiting...\033[0m")
                rclpy.shutdown()
                break

def main(args=None):
    rclpy.init(args=args)
    node = BotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\033[91m[INFO] bot_node interrupted by user\033[0m")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()