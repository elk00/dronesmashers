import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, ParamSet
from geometry_msgs.msg import PoseStamped
import socket
import struct
from threading import Thread
import time

STRUCT_FORMAT = '=IHIfff'
STRUCT_SIZE = struct.calcsize(STRUCT_FORMAT)

class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')
        self.get_logger().info("Initializing DroneController node...")

        # Initialize MAVROS services for arming, mode setting, and takeoff
        self.arm_service = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_service = self.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_service = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.get_logger().info("MAVROS services initialized.")

        # Publisher for motor speed and vision position data
        self.pub_motor = self.create_publisher(Float64, '/mavros/motor_speed', 20)
        self.pub_vision_pose = self.create_publisher(PoseStamped, '/mavros/vision_pose/pose', 10)
        self.get_logger().info("Publishers initialized.")

        # Ensure services are available
        self.get_logger().info("Waiting for MAVROS services to become available...")
        while not self.arm_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for arming service...')
        while not self.mode_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for set mode service...')

        # Set parameters to disable GPS requirements
        self.get_logger().info("Setting MAVROS parameters...")
        self.init_parameters()

        # Publish an initial static pose to initialize the vision pose topic
        self.publish_initial_pose()

        # Start publishing UWB data as vision data
        self.get_logger().info("Starting vision data publishing from UWB coordinates.")
        self.udp_thread = Thread(target=self.parse_udp_data)
        self.udp_thread.start()

        # Delay to allow vision data to initialize
        time.sleep(2)

        # Arm the drone and set mode
        self.set_mode('GUIDED')
        self.arm_drone()

    def init_parameters(self):
        # Sets parameters to bypass GPS and use vision data for position and altitude
        # self.set_param('EKF2_AID_MASK', integer_value=24)  # Enable vision position and yaw
        # self.set_param('EKF2_HGT_MODE', integer_value=3)   # Use vision data for altitude
        self.set_param('COM_ARM_WO_GPS', integer_value=1)  # Allow arming without GPS fix
        self.get_logger().info("Parameters set.")

    def set_param(self, param_id, integer_value=None, real_value=None):
        self.get_logger().info(f"Setting parameter {param_id}...")
        param_set_service = self.create_client(ParamSet, '/mavros/param/set')
        request = ParamSet.Request()
        request.param_id = param_id
        if integer_value is not None:
            request.value.integer = integer_value
        if real_value is not None:
            request.value.real = real_value

        future = param_set_service.call_async(request)
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
            self.get_logger().info(f'Switched to {mode} mode.')
        else:
            self.get_logger().error(f'Failed to set mode to {mode}.')

    def arm_drone(self):
        self.get_logger().info('Arming the drone...')
        request = CommandBool.Request()
        request.value = True
        future = self.arm_service.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None and future.result().success:
            self.get_logger().info('Drone armed successfully!')
            self.takeoff_drone()
        else:
            self.get_logger().error('Failed to arm the drone.')

    def takeoff_drone(self):
        self.get_logger().info('Initiating takeoff...')
        request = CommandTOL.Request()
        request.altitude = 2.0
        request.latitude = 0.0
        request.longitude = 0.0
        future = self.takeoff_service.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None and future.result().success:
            self.get_logger().info('Takeoff successful!')
            self.spin_motors()
        else:
            self.get_logger().error('Takeoff failed.')

    def spin_motors(self):
        self.get_logger().info('Starting motors...')
        msg = Float64()
        msg.data = 1000.0  # Set a low RPM
        self.pub_motor.publish(msg)
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=10))
        self.land_drone()

    def land_drone(self):
        self.get_logger().info('Landing the drone...')
        request = CommandTOL.Request()
        request.altitude = 0.0
        request.latitude = 0.0
        request.longitude = 0.0
        future = self.takeoff_service.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None and future.result().success:
            self.get_logger().info('Landing successful!')
            self.stop_motors()
        else:
            self.get_logger().error('Failed to land the drone.')

    def stop_motors(self):
        self.get_logger().info('Stopping motors...')
        msg = Float64()
        msg.data = 0.0
        self.pub_motor.publish(msg)
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

    def publish_initial_pose(self):
        # Publish an initial static pose to initialize the vision pose topic
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = 0.0
        pose_msg.pose.position.y = 0.0
        pose_msg.pose.position.z = 1.0  # Example altitude
        self.pub_vision_pose.publish(pose_msg)
        self.get_logger().info("Published initial static vision pose.")

    def parse_udp_data(self):
        # Open UDP socket to listen for UWB data
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(('127.0.0.1', 8000))
        self.get_logger().info('Listening for UDP data on 127.0.0.1:8000...')

        while rclpy.ok():
            data, addr = sock.recvfrom(STRUCT_SIZE)
            if len(data) == STRUCT_SIZE:
                parsed_data = struct.unpack(STRUCT_FORMAT, data)
                _, _, drone_id, drone_x, drone_y, drone_z = parsed_data
                self.get_logger().info(f'Received coordinates: ID={drone_id}, X={drone_x}, Y={drone_y}, Z={drone_z}')

                # Create and publish vision pose
                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = "map"
                pose_msg.pose.position.x = drone_x
                pose_msg.pose.position.y = drone_y
                pose_msg.pose.position.z = drone_z

                self.pub_vision_pose.publish(pose_msg)
                self.get_logger().info(f'Published vision pose: x={drone_x}, y={drone_y}, z={drone_z}')
            else:
                self.get_logger().error('Received incomplete data packet')

def main(args=None):
    rclpy.init(args=args)
    drone_controller = DroneController()
    rclpy.spin(drone_controller)
    drone_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

