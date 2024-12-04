import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from rclpy.qos import QoSProfile, ReliabilityPolicy

import time


class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')

        self.arm_service = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_service = self.create_client(SetMode, '/mavros/set_mode')

        self.position_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)

        # Initialize local position data
        self.current_position = None

        # Subscribe to local_position/pose topic
        self.local_position_subscriber = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.local_position_callback,
            qos_profile=QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                depth=10)
        )

        # Ensure the services are available
        while not self.arm_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for arming service...')

        while not self.mode_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for set mode service...')

        self.arm_drone()

    def local_position_callback(self, msg):
        """Callback to store the current local position."""
        self.current_position = msg
        self.get_logger().debug(f'Received local position: {msg.pose.position}')

    def arm_drone(self):
        self.set_mode('GUIDED')
        self.get_logger().info('Arming the drone...')
        request = CommandBool.Request()
        request.value = True

        future = self.arm_service.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None and future.result().success:
            self.get_logger().info('Drone armed successfully!')
            time.sleep(0.5)  # Wait to stabilize
            self.takeoff_drone()
        else:
            self.get_logger().error('Failed to arm the drone.')

    def set_mode(self, mode):
        request = SetMode.Request()
        request.custom_mode = mode

        future = self.mode_service.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            if future.result().mode_sent:
                self.get_logger().info(f'Switched to {mode} mode.')
            else:
                self.get_logger().error(f'Failed to set mode to {mode}. Response: {future.result()}')
        else:
            self.get_logger().error('Service call failed.')

    def takeoff_drone(self):
        """Takeoff using position control."""
        if self.current_position is None:
            self.get_logger().error('No local position data received. Cannot take off.')
            return

        self.get_logger().info('Taking off to a target height...')
        target_altitude = self.current_position.pose.position.z + 1.0  # Adjust as needed

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        pose.pose.position.x = self.current_position.pose.position.x
        pose.pose.position.y = self.current_position.pose.position.y
        pose.pose.position.z = target_altitude
        pose.pose.orientation.w = 1.0

        # Publish position setpoints
        start_time = time.time()
        while time.time() - start_time < 5:  # Maintain the setpoint for 5 seconds
            self.position_pub.publish(pose)
            time.sleep(0.1)

        self.get_logger().info('Reached takeoff height.')
        self.navigate_to_waypoint(3.0, 3.0, target_altitude)  # Example waypoint

    def navigate_to_waypoint(self, x, y, z):
        """Navigate to a specific waypoint."""
        self.get_logger().info(f'Navigating to waypoint: ({x}, {y}, {z})')

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0

        # Publish position setpoints
        start_time = time.time()
        while time.time() - start_time < 10:  # Maintain the setpoint for 10 seconds
            self.position_pub.publish(pose)
            time.sleep(0.1)

        self.get_logger().info('Reached the waypoint.')
        self.land_drone()

    def land_drone(self):
        """Land the drone."""
        self.get_logger().info('Landing the drone...')
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        pose.pose.position.x = self.current_position.pose.position.x
        pose.pose.position.y = self.current_position.pose.position.y
        pose.pose.position.z = 0.0  # Land to ground level
        pose.pose.orientation.w = 1.0

        # Publish landing setpoint
        start_time = time.time()
        while time.time() - start_time < 5:  # Maintain the setpoint for 5 seconds
            self.position_pub.publish(pose)
            time.sleep(0.1)

        self.disarm_drone()

    def disarm_drone(self):
        self.get_logger().info('Disarming the drone...')
        request = CommandBool.Request()
        request.value = False

        future = self.arm_service.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None and future.result().success:
            self.get_logger().info('Drone disarmed successfully!')
        else:
            self.get_logger().error('Failed to disarm the drone.')


def main(args=None):
    rclpy.init(args=args)
    drone_controller = DroneController()
    rclpy.spin(drone_controller)
    drone_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

