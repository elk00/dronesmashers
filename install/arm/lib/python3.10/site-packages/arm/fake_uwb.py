import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
import threading
import sys
import termios
import tty


class FakeUWBSimulator(Node):
    def __init__(self):
        super().__init__('fake_uwb_simulator')

        # UWB Publisher
        self.uwb_pub = self.create_publisher(PoseStamped, '/vision_pose/pose', 10)

        # Timer for publishing data
        self.timer = self.create_timer(0.1, self.publish_fake_uwb)  # Publish at 10 Hz

        # Initial UWB position
        self.uwb_position = {'x': 0.0, 'y': 0.0, 'z': 0.0}

        # Start a separate thread for keyboard input
        threading.Thread(target=self.keyboard_control, daemon=True).start()

    def publish_fake_uwb(self):
        """Publishes the fake UWB position data."""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()  # Set current time
        msg.header.frame_id = 'map'  # Ensure this matches your setup's frame

        # Set the current UWB position
        msg.pose.position.x = self.uwb_position['x']
        msg.pose.position.y = self.uwb_position['y']
        msg.pose.position.z = self.uwb_position['z']

        # No orientation change for now
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0

        self.uwb_pub.publish(msg)
        # Only display x, y, z values on the screen
        print(f"x: {self.uwb_position['x']:.2f}, y: {self.uwb_position['y']:.2f}, z: {self.uwb_position['z']:.2f}", end='\r')

    def keyboard_control(self):
        """Allows the user to control the UWB position using the keyboard."""
        print("Use WASD for x/y control, QE for z control. Press ESC to quit.")

        def get_key():
            """Reads a single key press."""
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            try:
                tty.setraw(sys.stdin.fileno())
                key = sys.stdin.read(1)
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            return key

        while True:
            key = get_key()
            if key == '\x1b':  # ESC key to quit
                print("\nExiting UWB Simulator.")
                rclpy.shutdown()
                break
            elif key == 'w':  # Move forward in y
                self.uwb_position['y'] += 0.1
            elif key == 's':  # Move backward in y
                self.uwb_position['y'] -= 0.1
            elif key == 'a':  # Move left in x
                self.uwb_position['x'] -= 0.1
            elif key == 'd':  # Move right in x
                self.uwb_position['x'] += 0.1
            elif key == 'q':  # Move up in z
                self.uwb_position['z'] += 0.1
            elif key == 'e':  # Move down in z
                self.uwb_position['z'] -= 0.1
            else:
                print("Invalid key! Use WASD for x/y control, QE for z control. Press ESC to quit.")


def main(args=None):
    rclpy.init(args=args)
    node = FakeUWBSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

