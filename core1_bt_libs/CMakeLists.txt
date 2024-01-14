cmake_minimum_required(VERSION 3.8)
project(core1_bt_libs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

file(GLOB_RECURSE SRCS src/*.cpp)
ament_auto_add_library(${PROJECT_NAME} SHARED ${SRCS})

ament_auto_package()
