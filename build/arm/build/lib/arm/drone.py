import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
import socket
import struct

STRUCT_FORMAT = '=IHIfff'
STRUCT_SIZE = struct.calcsize(STRUCT_FORMAT)

class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')

        self.arm_service = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_service = self.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_service = self.create_client(CommandTOL, '/mavros/cmd/takeoff')

        self.pub = self.create_publisher(Float64, '/mavros/motor_speed', 20)

        # Ensure the services are available
        while not self.arm_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for arming service...')

        while not self.mode_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for set mode service...')

        self.arm_drone()

    def arm_drone(self):
        self.set_mode('GUIDED_NOGPS')
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
        self.get_logger().info('Taking off to a short height...')
        request = CommandTOL.Request()
        request.altitude = 2.0  # Set desired altitude in meters
        request.latitude = 0.0   # Use current latitude
        request.longitude = 0.0  # Use current longitude

        future = self.takeoff_service.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None and future.result().success:
            self.get_logger().info('Takeoff successful!')
            self.spin_motors()
        else:
            self.get_logger().error('Failed to take off.')

    def spin_motors(self):
        self.get_logger().info('Starting motors...')
        msg = Float64()
        msg.data = 1000.0  # Set a low RPM (adjust as needed)

        # Publish the motor speed
        self.pub.publish(msg)

        # Spin for 10 seconds
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=100))

        self.land_drone()

    def land_drone(self):
        self.get_logger().info('Landing the drone...')
        request = CommandTOL.Request()
        request.altitude = 0.0  # Land to ground level
        request.latitude = 0.0   # Use current latitude
        request.longitude = 0.0  # Use current longitude

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
        msg.data = 0.0  # Stop the motors
        self.pub.publish(msg)

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

    def parse_udp_data(self):
        # Create a UDP socket to listen for incoming data on the loopback address
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(('127.0.0.1', 8000))
        self.get_logger().info('Listening for UDP data on 127.0.0.1:8000...')

        while True:
            # Receive data
            data, addr = sock.recvfrom(STRUCT_SIZE)

            if len(data) == STRUCT_SIZE:
                # Unpack the received data according to the STRUCT_FORMAT
                parsed_data = struct.unpack(STRUCT_FORMAT, data)  
                message_length, message_type, drone_id, drone_x, drone_y, drone_z = parsed_data
                print(drone_id, drone_x, drone_y, drone_z)
            else:
                print("wrong data received")
                self.get_logger().error('Received incomplete data packet.')

def main(args=None):
    rclpy.init(args=args)
    drone_controller = DroneController()

    # Parse UDP data in a separate thread or asynchronously
    from threading import Thread
    udp_thread = Thread(target=drone_controller.parse_udp_data)
    udp_thread.start()

    rclpy.spin(drone_controller)

    # Shutdown ROS and cleanup
    udp_thread.join()
    drone_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
