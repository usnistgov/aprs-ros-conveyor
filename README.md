# APRS ROS2 Conveyor Package 

ROS2 package for controlling the conveyor belt in the NIST APRS Lab

## Package Descriptions

* `conveyor_interfaces`: Contains ROS msgs and srvs for inter node communication
* `conveyor_controller`: Contains source code for the ROS2 node to enable the conveyor and modify its speed and direction


## Installation

* Create a container containing ROS2 on docker desktop
* Run container in VSCode
* Clone this package with:

    `git clone [https://github.com](https://github.com/usnistgov/aprs-ros-conveyor.git)`

  
## Instructions for Running the Software

1. Cd to host directory
2. Build the package with:
    `colcon build`
3. Source the package with:
   `source install/setup.bash`
4. Run the node with:
   `ros2 run conveyor_controller conveyor_controller`
5. In a new terminal, set the variables with:
   `ros2 topic echo /conveyor/state --once`
6. In the same terminal, enable the conveyor with:
    `ros2 service call /conveyor/enable conveyor_interfaces/srv/EnableConveyor "{enable: true}"`
7. Set the speed and direction with:
    `ros2 service call /conveyor/set_state conveyor_interfaces/srv/SetConveyorState "{speed: 50.0, direction: 1}"`
8. Speed must be 0 < x < 100
9. Direction must be 0 or 1 for Forward and Reverse respectively
10. In a new terminal, get topic status with:
     `ros2 topic echo /conveyor/state`




