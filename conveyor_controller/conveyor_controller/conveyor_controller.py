# Node that controls Conveyor Speed/Direction/ON/OFF

import rclpy
from rclpy.node import Node
from pymodbus.client import ModbusTcpClient
from conveyor_interfaces.msg import ConveyorState
from conveyor_interfaces.srv import EnableConveyor, SetConveyorState

class ConveyorController(Node):
    def __init__(self):
        super().__init__('conveyor_controller')

        #create publisher
        self.pub_state = self.create_publisher(ConveyorState, "conveyor/state", 10) 
        self.publish_timer = self.create_timer(1, self.pub_state_cb)

        #create services
        self.enable_conveyor = self.create_service(EnableConveyor, "conveyor/enable", self.enable_conveyor_cb)
        self.set_conveyor = self.create_service(SetConveyorState, "conveyor/set_state", self.set_conveyor_state_cb)

        #set variables 
        self.direction = ConveyorState.FORWARD
        self.speed = 0.0
        self.enabled = False 

        self.direction_options = [ConveyorState.FORWARD, ConveyorState.REVERSE]

        #connect to modbus
        self.client = ModbusTcpClient('192.168.1.50', port='502')
        if not self.client.connect():
            self.client.connect()


        #define publisher callback
    def pub_state_cb(self):
        state_msg = ConveyorState()
        state_msg.enabled = self.enabled
        state_msg.speed = self.speed
        state_msg.direction = self.direction
        self.pub_state.publish(state_msg)


        #define on/off callback
    def enable_conveyor_cb(self, request: EnableConveyor.Request, response: EnableConveyor.Response):
        if request.enable and self.enabled:
            self.get_logger().warn("Conveyor is already enabled")
            response.success = False
            response.message = "Conveyor is already enabled"
            return response
        
        if not request.enable and not self.enabled:
            self.get_logger().warn("Conveyor is already disabled")
            response.success = False
            response.message = "Conveyor is already disabled"
            return response

        if request.enable:
            self.enabled = request.enable
            self.client.write_register(0x8001, self.direction)
            self.client.write_register(0x8002, int(self.speed * 327.68))
            self.client.write_register(0x8000, self.enabled)
            response.success = True
            response.message = "Conveyor On"
            return response
        
        if not request.enable:
            self.enabled = request.enable
            self.client.write_register(0x8000, self.enabled)
            self.speed = 0.0
            self.direction = 0
            response.success = True
            response.message = "Conveyor Off"
            return response


        #define set state callback (speed/direction)
    def set_conveyor_state_cb(self, request: SetConveyorState.Request, response: SetConveyorState.Response):
        if not self.enabled:
            self.get_logger().error("Conveyor must be enabled first")
            response.success = False
            response.message = "Conveyor must be enabled first"
            return response

        self.get_logger().info(f"Speed: {request.speed}, direction: {request.direction}")
        
        if request.speed < 0 or request.speed > 100:
            self.get_logger().warn("Speed must be between 0 and 100")
            response.success = False
            response.message = "Speed must be between 0 and 100"
            return response
        
        if request.direction not in self.direction_options:
            self.get_logger().warn("Direction should be 0 or 1")
            response.success = False
            response.message = "Direction should be 0 or 1"
            return response
        
        self.speed = request.speed
        self.direction = request.direction
        self.client.write_register(0x8001, self.direction)
        self.client.write_register(0x8002, int(self.speed * 327.68))
        response.success = True
        response.message = f"Speed: {self.speed}, direction: {self.direction}"
        return response
   
def main(args=None):
    rclpy.init(args=args)
    conveyor = ConveyorController()
    conveyor.get_logger().info("Conveyor controller node is running")

    try: 
        rclpy.spin(conveyor)
    except KeyboardInterrupt as ex:
        pass

    conveyor.client.close() 

if __name__ == "__main__":
    main()