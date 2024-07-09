cmake_minimum_required(VERSION 3.5)
project(ldlidar_stl_ros2)

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

message(STATUS "${LIB_NAME} Ros version : $ENV{ROS_DISTRO}")
if ($ENV{ROS_DISTRO} MATCHES "iron")
  add_definitions(-DUSE_IRON_VERSION)
  set(USE_ROS1_MIDDLE_PRICE OFF)
  find_package(spdlog REQUIRED)
  set(LIBSPDLOG spdlog::spdlog)
elseif ($ENV{ROS_DISTRO} MATCHES "humble")
  add_definitions(-DUSE_HUMBLE_VERSION)
  set(USE_ROS1_MIDDLE_PRICE OFF)
  find_package(spdlog REQUIRED)
  set(LIBSPDLOG spdlog::spdlog)
elseif ($ENV{ROS_DISTRO} MATCHES "galactic")
  add_definitions(-DUSE_GALACTIC_VERSION)
  set(USE_ROS1_MIDDLE_PRICE OFF)
  find_package(spdlog REQUIRED)
  set(LIBSPDLOG spdlog::spdlog)
elseif ($ENV{ROS_DISTRO} MATCHES "foxy")
  add_definitions(-DUSE_FOXY_VERSION)
  set(USE_ROS1_MIDDLE_PRICE OFF)
  find_package(spdlog REQUIRED)
  set(LIBSPDLOG spdlog::spdlog)
elseif ($ENV{ROS_DISTRO} MATCHES "eloquent")
  add_definitions(-DUSE_ELOQUENT_VERSION)
  set(USE_ROS1_MIDDLE_PRICE OFF)
  include_directories(include/spdlog)
elseif ($ENV{ROS_DISTRO} MATCHES "dashing")
  add_definitions(-DUSE_DASHING_VERSION)
  set(USE_ROS1_MIDDLE_PRICE OFF)
  include_directories(include/spdlog)
elseif ($ENV{ROS_DISTRO} MATCHES "noetic")
  add_definitions(-DUSE_ROS_NORTIC_VERSION)
  set(USE_ROS1_MIDDLE_PRICE ON)
  include_directories(include/spdlog)
elseif ($ENV{ROS_DISTRO} MATCHES "melodic")
  add_definitions(-DUSE_ROS_MELODIC_VERSION)
  set(USE_ROS1_MIDDLE_PRICE ON)
  include_directories(include/spdlog)
endif()

# Bin and Install
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/ldlidar_driver/include
)

file(GLOB  DRIVER_SRC ${CMAKE_CURRENT_SOURCE_DIR}/include/ldlidar_driver/*.cpp)

if(USE_ROS1_MIDDLE_PRICE)
######################## 使用ROS1 作为中间件 ###########################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node  ${CMAKE_CURRENT_SOURCE_DIR}/src/demo.cpp ${DRIVER_SRC})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${LIBSPDLOG})
add_dependencies(${PROJECT_NAME}_node
  roscpp
  sensor_msgs)

install(TARGETS ${PROJECT_NAME}_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

#######################################################################
else()
######################## 使用ROS2 作为中间件 ###########################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

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

add_executable(${PROJECT_NAME}_node  ${CMAKE_CURRENT_SOURCE_DIR}/src/demo.cpp ${DRIVER_SRC})
ament_target_dependencies(${PROJECT_NAME}_node rclcpp sensor_msgs)
target_link_libraries(${PROJECT_NAME}_node pthread ${LIBSPDLOG})

# Install
install(TARGETS ${PROJECT_NAME}_node
 DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch rviz2 params
 DESTINATION share/${PROJECT_NAME}/
)

ament_package()
#######################################################################
endif()
