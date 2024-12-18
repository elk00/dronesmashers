import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, StreamRate
from rclpy.qos import QoSProfile, ReliabilityPolicy
import math
import time


class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')

        # MAVROS services for arming, mode switching, takeoff, landing, and setting stream rate
        self.arm_service = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_service = self.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_service = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.land_service = self.create_client(CommandTOL, '/mavros/cmd/land')
        self.stream_rate_service = self.create_client(StreamRate, '/mavros/set_stream_rate')

        # Publishers for GPS data
        self.gps_pub = self.create_publisher(NavSatFix, '/mavros/global_position/global', 10)

        # Subscribe to local position updates
        self.current_position = None
        self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.local_position_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )

        # Set an origin for GPS conversion (lat, lon, altitude)
        self.origin_lat = 1.3000  # Example latitude (Singapore)
        self.origin_lon = 103.8000  # Example longitude (Singapore)
        self.origin_alt = 10.0  # Example altitude in meters

        # Wait for services to become available
        self.wait_for_services()

        # Set the MAVROS stream rate
        self.set_stream_rate(stream_id=0, message_rate=50, on_off=True)

        # Start a timer to publish fake GPS data
        self.timer = self.create_timer(0.5, self.publish_fake_gps)

        self.get_logger().info('Drone Controller Node Initialized.')
        self.arm_drone()

    def wait_for_services(self):
        """Ensure all required MAVROS services are available."""
        while not self.arm_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for arming service...')
        while not self.mode_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for mode service...')
        while not self.takeoff_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for takeoff service...')
        while not self.land_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for land service...')
        while not self.stream_rate_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for stream rate service...')

    def set_stream_rate(self, stream_id, message_rate, on_off):
        """Set the MAVROS stream rate."""
        self.get_logger().info(f"Setting MAVROS stream rate: stream_id={stream_id},         rate={message_rate}, on_off={on_off}")
        stream_request = StreamRate.Request()
        stream_request.stream_id = stream_id
        stream_request.message_rate = message_rate
        stream_request.on_off = on_off

        future = self.stream_rate_service.call_async(stream_request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info("Stream rate set successfully.")
        else:
            self.get_logger().error("Failed to set stream rate.")


    def local_position_callback(self, msg):
        """Update current position from MAVROS."""
        self.current_position = msg.pose
        self.get_logger().debug(f"Current Position: {self.current_position}")

    def publish_fake_gps(self):
        """Convert local position to fake GPS data and publish it."""
        if not self.current_position:
            return

        # Convert local position to GPS
        x = self.current_position.position.x
        y = self.current_position.position.y
        z = self.current_position.position.z

        # Approximate conversions
        lat = self.origin_lat + (x / 111111)  # 1 degree latitude ~ 111,111 meters
        lon = self.origin_lon + (y / (111111 * math.cos(math.radians(self.origin_lat))))
        alt = self.origin_alt + z

        # Create and publish GPS message
        gps_msg = NavSatFix()
        gps_msg.latitude = lat
        gps_msg.longitude = lon
        gps_msg.altitude = alt
        gps_msg.header.stamp = self.get_clock().now().to_msg()
        gps_msg.header.frame_id = 'map'
        self.gps_pub.publish(gps_msg)

        self.get_logger().debug(f"Published Fake GPS: lat={lat}, lon={lon}, alt={alt}")

    def arm_drone(self):
        """Arm the drone and switch to GUIDED mode."""
        self.set_mode('GUIDED')
        self.get_logger().info('Arming the drone...')
        arm_request = CommandBool.Request()
        arm_request.value = True

        future = self.arm_service.call_async(arm_request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None and future.result().success:
            self.get_logger().info('Drone armed successfully.')
            if self.wait_for_gps():
                self.takeoff_drone()
            else:
                self.get_logger().error("Failed to receive valid GPS data. Aborting takeoff.")
        else:
            self.get_logger().error('Failed to arm the drone.')

    def wait_for_gps(self, timeout=10):
        """Wait for valid GPS data."""
        self.get_logger().info("Waiting for valid GPS data...")
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.current_position:
                self.get_logger().info("Valid GPS data received.")
                return True
            time.sleep(0.25)

        self.get_logger().error("Timeout waiting for GPS data.")
        return False

    def set_mode(self, mode):
        """Set the flight mode."""
        self.get_logger().info(f'Switching to {mode} mode...')
        mode_request = SetMode.Request()
        mode_request.custom_mode = mode

        future = self.mode_service.call_async(mode_request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None and future.result().mode_sent:
            self.get_logger().info(f'Mode switched to {mode}.')
        else:
            self.get_logger().error(f'Failed to set mode to {mode}.')

    def takeoff_drone(self):
        """Take off using CommandTOL."""
        if not self.current_position:
            self.get_logger().error('No position data available. Cannot take off.')
            return

        self.get_logger().info('Taking off using CommandTOL...')
        target_altitude = 0.8  # Desired altitude

        # Use current GPS location for takeoff
        lat = self.origin_lat
        lon = self.origin_lon
        alt = self.origin_alt

        # Prepare the CommandTOL request
        takeoff_request = CommandTOL.Request()
        takeoff_request.latitude = lat
        takeoff_request.longitude = lon
        takeoff_request.altitude = alt + target_altitude
        takeoff_request.yaw = 0.0
        takeoff_request.min_pitch = 0.0

        # Call the takeoff service
        future = self.takeoff_service.call_async(takeoff_request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None and future.result().success:
            self.get_logger().info('Takeoff command executed successfully.')
            if self.stabilize_after_takeoff(target_altitude):
                self.land_drone()  # Proceed to landing
            else:
                self.get_logger().error("Takeoff stabilization failed. Aborting.")
        else:
            self.get_logger().error(f'Takeoff command failed: {future.result()}')

    def stabilize_after_takeoff(self, target_altitude, timeout=10):
        """Wait for the drone to stabilize at the target altitude."""
        self.get_logger().info(f"Waiting for drone to reach altitude: {target_altitude} meters...")
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.current_position and abs(self.current_position.position.z - target_altitude) < 0.1:
                self.get_logger().info("Drone stabilized at target altitude.")
                return True
            time.sleep(0.5)

        self.get_logger().error("Drone failed to stabilize at target altitude.")
        return False

    def land_drone(self):
        """Land using CommandTOL."""
        self.get_logger().info('Landing using CommandTOL...')
        
        # Use current GPS location for landing
        lat = self.origin_lat
        lon = self.origin_lon

        # Prepare the CommandTOL request
        land_request = CommandTOL.Request()
        land_request.latitude = lat
        land_request.longitude = lon
        land_request.altitude = 0.0  # Land at ground level
        land_request.yaw = 0.0
        land_request.min_pitch = 0.0

        # Call the land service
        future = self.land_service.call_async(land_request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None and future.result().success:
            self.get_logger().info('Land command executed successfully.')
            self.disarm_drone()
        else:
            self.get_logger().error(f'Land command failed: {future.result()}')

    def disarm_drone(self):
        """Disarm the drone."""
        self.get_logger().info('Disarming the drone...')
        arm_request = CommandBool.Request()
        arm_request.value = False

        future = self.arm_service.call_async(arm_request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None and future.result().success:
            self.get_logger().info('Drone disarmed successfully.')
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

