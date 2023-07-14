import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String
from rcl_interfaces.srv import SetParameters
from pymodbus.client import ModbusTcpClient

class ConveyorController(Node):
    def __init__(self):
        super().__init__('conveyor_controller')

        # create publishers for topic status
        self.on_off_pub = self.create_publisher(Bool, 'conveyor/on_off', 10)
        self.speed_pub = self.create_publisher(Float32, 'conveyor/speed', 10)
        self.direction_pub = self.create_publisher(String, 'conveyor/direction', 10)

        # create service servers for actuating services
        self.set_speed_srv = self.create_service(SetParameters, 'conveyor/set_speed', self.set_speed_callback)
        self.set_direction_srv = self.create_service(SetParameters, 'conveyor/set_direction', self.set_direction_callback)
        self.enable_srv = self.create_service(SetParameters, 'conveyor/enable', self.enable_callback)

        # Connect to modbus client and initialize variables
        self.client = ModbusTcpClient('192.168.1.50', port='502')
        self.client.connect()
          

    def set_speed_callback(self, request, response):
        # set the speed of the conveyor
        speed = request.values[0]

        
        if speed < 0 or speed > 100:
            response.successful = False
            response.reason = "Invalid speed value. Speed must be between 0 and 100."
            return response

        # Call the method in modbus client to set the speed
        self.client.set_speed(speed)

        response.successful = True
        response.reason = f"Speed set to {speed}"
        return response

    def set_direction_callback(self, request, response):
        # set the direction of the conveyor
        direction = request.values[0].string_value

        valid_directions = ["forward", "backward"]
        if direction not in valid_directions:
            response.successful = False
            response.reason = "Invalid direction. Direction must be 'forward' or 'backward'."
            return response

        self.client.set_direction(direction)

        
        # Return the response indicating the success
        response.successful = True
        response.reason = f"Direction set to {direction}"
        return response

    def enable_callback(self, request, response):
        enable = request.values[0].bool_value

        # enable or disable the conveyor
        if enable:
            self.client.enable()
            response.reason = "Conveyor enabled"
        else:
            self.client.disable()
            response.reason = "Conveyor disabled"

        response.successful = True
        return response

    def update_topic_status(self):
        # Retrieve the current status from the modbus client
        get_status = self.client.get_status()
        speed = self.client.get_speed()
        direction = self.client.get_direction()

        # messages for on/off, speed, and direction
        on_off_msg = Bool()
        on_off_msg.data = get_status

        speed_msg = Float32()
        speed_msg.data = speed

        direction_msg = String()
        direction_msg.data = direction

        # Publish the messages
        self.on_off_pub.publish(on_off_msg)
        self.speed_pub.publish(speed_msg)
        self.direction_pub.publish(direction_msg)


def main(args=None):
    rclpy.init(args=args)
    controller = ConveyorController()

    # Create a timer to update the topic status
    timer_period = 1.0  
    timer = controller.create_timer(timer_period, controller.update_topic_status)

    rclpy.spin(controller)

    
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
