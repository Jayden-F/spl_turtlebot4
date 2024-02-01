cmake_minimum_required(VERSION 3.5)
project(commander_server)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


include_directories(
  include
)
install(DIRECTORY include/
  DESTINATION include/
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action  REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(irobot_create_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)

add_executable(turtlebot4_commander commander_server/turtlebot4_commander.cpp)
ament_target_dependencies(turtlebot4_commander
rclcpp
rclcpp_action
irobot_create_msgs
nav2_msgs
diagnostic_msgs
nav2_util
)

include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(turtlebot4_commander ${Boost_LIBRARIES})

install(TARGETS
turtlebot4_commander
DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS commander_server/turtlebot4_commander.py DESTINATION lib/${PROJECT_NAME})

ament_package()

