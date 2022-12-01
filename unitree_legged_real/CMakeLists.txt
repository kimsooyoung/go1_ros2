cmake_minimum_required(VERSION 3.5)
project(unitree_legged_real)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# # Default to C++14
# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 14)
# endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros2_unitree_legged_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)


message("-- CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
  set(ARCH amd64)
  set(LEGGED_SDK_DIR "/home/kimsooyoung/Documents/unitree_legged_sdk_v3.5.1")
else()
  set(ARCH arm64)
  set(LEGGED_SDK_DIR "/home/unitree/Unitree/sdk/unitree_legged_sdk")
endif()
  set(EXTRA_LIBS -pthread libunitree_legged_sdk_${ARCH}.so lcm)

set(CMAKE_CXX_FLAGS "-O3")

include_directories(
    include
    ${LEGGED_SDK_DIR}/include
)

link_directories(${LEGGED_SDK_DIR}/lib)

add_executable(ros2_udp src/ros2_udp.cpp)
target_link_libraries(ros2_udp ${EXTRA_LIBS})
ament_target_dependencies(ros2_udp rclcpp ros2_unitree_legged_msgs)

add_executable(ros2_walk_example src/ros2_walk_example.cpp)
target_link_libraries(ros2_walk_example ${EXTRA_LIBS})
ament_target_dependencies(ros2_walk_example rclcpp ros2_unitree_legged_msgs)

add_executable(ros2_position_example src/ros2_position_example.cpp)
target_link_libraries(ros2_position_example ${EXTRA_LIBS})
ament_target_dependencies(ros2_position_example rclcpp ros2_unitree_legged_msgs)

add_executable(ros_state_helper src/ros_state_helper.cpp)
target_link_libraries(ros_state_helper ${EXTRA_LIBS})
ament_target_dependencies(ros_state_helper
  tf2  
  rclcpp
  tf2_ros
  nav_msgs
  sensor_msgs
  geometry_msgs
  ros2_unitree_legged_msgs
)


install(TARGETS
    ros2_udp
    ros_state_helper
    ros2_walk_example
    ros2_position_example
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
