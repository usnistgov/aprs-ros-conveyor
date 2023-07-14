#!/usr/bin/env python
import rclpy
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import Trigger, SetBool, SetFloat32, SetString


from pymodbus.client import ModbusTcpClient
from time import sleep
import sys

def main(direction, speed, duration):
    client = ModbusTcpClient('192.168.1.50', port='502')
    print(client.connect())

    # Set conveyor direction
    client.write_register(0x8001, direction)

    # Set conveyor speed
    client.write_register(0x8002, int(speed * 327.68))

    # Turn conveyor on
    client.write_register(0x8000, 1)

    sleep(duration)

    # Turn conveyor off
    client.write_register(0x8000, 0)

    client.close()


if __name__ == "__main__":
    print(sys.argv)
    direction = int(sys.argv[1])
    speed = int(sys.argv[2])
    duration = int(sys.argv[3])

    if (direction != 0 and direction != 1):
        print("Direction must be 0 or 1")
        exit()

    if not(0 <= speed <= 100):
        print("Speed must be between 0 and 100")
        exit()

    if duration > 20:
        print("Duration must be under 20 seconds")
        duration = 20

    main(direction, speed, duration)