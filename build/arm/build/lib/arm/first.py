import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, ParamSet
import socket
import struct

STRUCT_FORMAT = '=IHIfff'  # Keep the original format with three floats (x, y, z)
STRUCT_SIZE = struct.calcsize(STRUCT_FORMAT)

class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')

        # Initialize MAVROS services for arming, mode setting, takeoff, and parameter setting
        self.arm_service = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_service = self.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_service = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.param_set_service = self.create_client(ParamSet, '/mavros/param/set')

        # Initialize publishers for motor speed and fake vision data
        self.pub_motor = self.create_publisher(Float64, '/mavros/motor_speed', 20)
        self.pub_vision = self.create_publisher(PoseStamped, '/mavros/vision_pose/pose', 10)

        self.get_logger().info("Initializing DroneController: Waiting for services...")

        # Ensure all required services are available
        while not self.arm_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for arming service...')
        while not self.mode_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for set mode service...')

        # Publish fake vision data continuously to meet position requirements
        self.get_logger().info("Publishing fake vision data from UWB coordinates...")
        self.publish_fake_vision()

        # Set the mode to GUIDED and arm the drone
        self.set_mode('GUIDED')
        self.arm_drone()
    
    def set_preflight_param(self, param_id, integer_value=None, real_value=None):
        request = ParamSet.Request()
        request.param_id = param_id
        if integer_value is not None:
            request.value.integer = integer_value
        elif real_value is not None:
            request.value.real = real_value

        future = self.param_set_service.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None and future.result().success:
            self.get_logger().info(f'Successfully set {param_id} to {integer_value or real_value}')
        else:
            self.get_logger().error(f'Failed to set {param_id}')

    def set_mode(self, mode):
        self.get_logger().info(f"Setting mode to {mode}...")
        request = SetMode.Request()
        request.custom_mode = mode
        future = self.mode_service.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None and future.result().mode_sent:
            self.get_logger().info(f"Mode successfully set to {mode}.")
        else:
            self.get_logger().error(f"Failed to set mode to {mode}.")

    def arm_drone(self):
        self.get_logger().info("Attempting to arm the drone...")
        request = CommandBool.Request()
        request.value = True
        future = self.arm_service.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None and future.result().success:
            self.get_logger().info("Drone armed successfully!")
            self.takeoff_drone()
        else:
            self.get_logger().error("Failed to arm the drone. Please check system status.")

    def takeoff_drone(self):
        self.get_logger().info("Initiating takeoff sequence...")
        request = CommandTOL.Request()
        request.altitude = 2.0  # Set altitude to 2 meters
        request.latitude, request.longitude = self.get_fake_coordinates()

        future = self.takeoff_service.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None and future.result().success:
            self.get_logger().info("Takeoff successful!")
            self.spin_motors()
        else:
            self.get_logger().error("Takeoff failed. Check system status.")

    def spin_motors(self):
        self.get_logger().info("Starting motors...")
        msg = Float64()
        msg.data = 1000.0  # Set RPM for low speed
        self.pub_motor.publish(msg)

        self.get_logger().info("Spinning motors for 10 seconds...")
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=10))
        self.land_drone()

    def land_drone(self):
        self.get_logger().info("Initiating landing sequence...")
        request = CommandTOL.Request()
        request.altitude = 0.0  # Land at ground level
        request.latitude, request.longitude = self.get_fake_coordinates()

        future = self.takeoff_service.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None and future.result().success:
            self.get_logger().info("Landing successful!")
            self.stop_motors()
        else:
            self.get_logger().error("Landing failed. Check system status.")

    def stop_motors(self):
        self.get_logger().info("Stopping motors...")
        msg = Float64()
        msg.data = 0.0  # Stop motor speed
        self.pub_motor.publish(msg)
        self.disarm_drone()

    def disarm_drone(self):
        self.get_logger().info("Disarming the drone...")
        request = CommandBool.Request()
        request.value = False
        future = self.arm_service.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None and future.result().success:
            self.get_logger().info("Drone disarmed successfully.")
        else:
            self.get_logger().error("Failed to disarm the drone.")

    def get_fake_coordinates(self):
        self.get_logger().info("Fetching fake coordinates from UWB data...")
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(('127.0.0.1', 8000))
        data, addr = sock.recvfrom(STRUCT_SIZE)

        parsed_data = struct.unpack(STRUCT_FORMAT, data)
        drone_x, drone_y, drone_z = parsed_data[3], parsed_data[4], parsed_data[5]
        
        latitude = 1.0 + (drone_x / 1e6)
        longitude = 103.0 + (drone_y / 1e6)
        altitude = drone_z
        self.get_logger().info(f"Fake coordinates set: latitude={latitude}, longitude={longitude}, altitude={altitude}")
        
        return latitude, longitude

    def publish_fake_vision(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(('127.0.0.1', 8000))

        while rclpy.ok():
            data, addr = sock.recvfrom(STRUCT_SIZE)
            parsed_data = struct.unpack(STRUCT_FORMAT, data)
            drone_x, drone_y, drone_z = parsed_data[3], parsed_data[4], parsed_data[5]

            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "map"  # Set an appropriate frame
            pose_msg.pose.position.x = drone_x
            pose_msg.pose.position.y = drone_y
            pose_msg.pose.position.z = drone_z

            self.pub_vision.publish(pose_msg)
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.1))  # Publish at 10 Hz

def main(args=None):
    rclpy.init(args=args)
    drone_controller = DroneController()
    
    rclpy.spin(drone_controller)

    drone_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

