cmake_minimum_required(VERSION 3.8)
project(core1_bt_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(main src/main.cpp)

ament_auto_add_executable(example_action_server src/example_action_server.cpp)

ament_auto_package(INSTALL_TO_SHARE
  trees
)
