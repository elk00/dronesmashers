import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode

class ArmCopter(Node):
    def __init__(self):
        super().__init__('arm_copter')
        self.arm_service = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_service = self.create_client(SetMode, '/mavros/set_mode')

        while not self.arm_service.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Waiting for arming service...')
        
        while not self.set_mode_service.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Waiting for set mode service...')

    def arm(self):
        request = CommandBool.Request()
        request.value = True  # Set to True to arm
        future = self.arm_service.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info('Arming result: %s' % future.result().success)
        else:
            self.get_logger().error('Failed to call arming service.')

    def set_offboard_mode(self):
    	request = SetMode.Request()  # Use SetMode.Request() to create a request
    	request.custom_mode = 'GUIDED'  # Set to 'OFFBOARD' mode
    	future = self.set_mode_service.call_async(request)
    	rclpy.spin_until_future_complete(self, future)

    	if future.result() is not None:
        	self.get_logger().info('Set mode result: %s' % future.result().mode_sent)
    	else:
        	self.get_logger().error('Failed to set mode.')


def main(args=None):
    rclpy.init(args=args)
    arm_copter = ArmCopter()

    # First, set the mode to OFFBOARD
    arm_copter.set_offboard_mode()

    # Then, arm the copter
    arm_copter.arm()

    arm_copter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

