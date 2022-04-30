cmake_minimum_required(VERSION 3.5)
project(evaluation_scenarios)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(DEFAULT_PATH
    src/
)

set(DEFAULT_ROBOT_SOURCES
    ${DEFAULT_PATH}default_robot_node.cpp
)

set(SIMPLE_PATH
    ${DEFAULT_PATH}simple/
)

set(ADVANCED_PATH
    ${DEFAULT_PATH}advanced/
)

set(SET_POSITION_SOURCES
    ${SIMPLE_PATH}set_position/set_position_node.cpp
    ${DEFAULT_ROBOT_SOURCES}
)

set(SET_VELOCITY_SOURCES
    ${SIMPLE_PATH}set_velocity/set_velocity_node.cpp
    ${DEFAULT_ROBOT_SOURCES}
)

set(SET_POWER_SOURCES
    ${SIMPLE_PATH}set_power/set_power_node.cpp
    ${DEFAULT_ROBOT_SOURCES}
)

## SIMPLE

# Set position executables

add_executable(set_position_turn_left
    ${SIMPLE_PATH}set_position/turn_left.cpp
    ${SET_POSITION_SOURCES}
)

add_executable(set_position_turn_right
    ${SIMPLE_PATH}set_position/turn_right.cpp
    ${SET_POSITION_SOURCES}
)

# Set velocity executables

add_executable(set_velocity_turn_left
    ${SIMPLE_PATH}set_velocity/turn_left.cpp
    ${SET_VELOCITY_SOURCES}
)

add_executable(set_velocity_turn_right
    ${SIMPLE_PATH}set_velocity/turn_right.cpp
    ${SET_VELOCITY_SOURCES}
)

# Set power executables

add_executable(set_power_turn_left
    ${SIMPLE_PATH}set_power/turn_left.cpp
    ${SET_POWER_SOURCES}
)

add_executable(set_power_turn_right
    ${SIMPLE_PATH}set_power/turn_right.cpp
    ${SET_POWER_SOURCES}
)

ament_target_dependencies(set_position_turn_left rclcpp std_msgs)
ament_target_dependencies(set_position_turn_right rclcpp std_msgs)

ament_target_dependencies(set_velocity_turn_left rclcpp std_msgs)
ament_target_dependencies(set_velocity_turn_right rclcpp std_msgs)

ament_target_dependencies(set_power_turn_left rclcpp std_msgs)
ament_target_dependencies(set_power_turn_right rclcpp std_msgs)


## ADVANCES

# Light Sensor

add_executable(light_sensor
    ${ADVANCED_PATH}light/robot.cpp
    ${DEFAULT_ROBOT_SOURCES}
)

ament_target_dependencies(light_sensor rclcpp std_msgs)

# Distance Sensor

add_executable(distance_sensor
    ${ADVANCED_PATH}distance/robot.cpp
    ${DEFAULT_ROBOT_SOURCES}
)   

ament_target_dependencies(distance_sensor rclcpp std_msgs)

# Touch Sensor

add_executable(touch_sensor
    ${ADVANCED_PATH}touch/robot.cpp
    ${DEFAULT_ROBOT_SOURCES}
)   

ament_target_dependencies(touch_sensor rclcpp std_msgs)

# Compass Sensor

add_executable(compass_sensor
    ${ADVANCED_PATH}compass/robot.cpp
    ${DEFAULT_ROBOT_SOURCES}
)   

ament_target_dependencies(compass_sensor rclcpp std_msgs)


install(TARGETS
    # SIMPLE
    set_position_turn_left
    set_position_turn_right

    set_velocity_turn_left
    set_velocity_turn_right

    set_power_turn_left
    set_power_turn_right

    # ADVANCED
    light_sensor
    distance_sensor
    touch_sensor
    compass_sensor

    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
