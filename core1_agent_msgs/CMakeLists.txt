cmake_minimum_required(VERSION 3.5)
project(core1_agent_msgs)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(core1_agent_msgs
    "action/ExampleTimer.action"
    "action/Trigger.action")

ament_export_dependencies(rosidl_default_runtime)
ament_package()
