cmake_minimum_required(VERSION 3.8)
project(conveyor_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rosidl_default_generators REQUIRED)
set(msg_files
  "msg/ConveyorState.msg")
  set(srv_files
  "srv/EnableConveyor.srv"
  "srv/SetConveyorState.srv")

  rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files})

ament_export_dependencies(rosidl_default_runtime)





# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


ament_package()
