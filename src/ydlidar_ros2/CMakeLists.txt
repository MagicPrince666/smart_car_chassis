cmake_minimum_required(VERSION 3.5)

project(ydlidar)

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

find_package(rmw REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(spdlog REQUIRED)

set(SDK_PATH "./sdk/")

FILE(GLOB SDK_SRC 
  "${SDK_PATH}/src/impl/unix/*.cpp"
  "${SDK_PATH}/src/*.cpp"
  "${SDK_PATH}/src/*.c"
)

include_directories(
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/sdk/include
  ${PROJECT_SOURCE_DIR}/sdk/src
)

if(USE_ROS1_MIDDLE_PRICE)
######################## 使用ROS1 作为中间件 ###########################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  visualization_msgs
  geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(ydlidar_node src/ydlidar_node.cpp  ${SDK_SRC})
target_link_libraries(ydlidar_node ${catkin_LIBRARIES} ${LIBSPDLOG})
add_dependencies(ydlidar_node
  roscpp
  std_msgs
  geometry_msgs
  visualization_msgs)

add_executable(ydlidar_client src/ydlidar_client.cpp)
target_link_libraries(ydlidar_client ${catkin_LIBRARIES})
add_dependencies(ydlidar_client
  roscpp
  std_msgs
  geometry_msgs
  visualization_msgs)

install(TARGETS ydlidar_node ydlidar_client
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
#######################################################################
else()
######################## 使用ROS2 作为中间件 ###########################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(
    ${roscpp_INCLUDE_DIRS} 
    ${roslib_INCLUDE_DIRS} 
)

add_executable(ydlidar_node src/ydlidar_node.cpp  ${SDK_SRC})
target_link_libraries(ydlidar_node ${LIBSPDLOG})
ament_target_dependencies(ydlidar_node
  "rclcpp"
  "sensor_msgs"
  "visualization_msgs"
  "geometry_msgs"
)

add_executable(ydlidar_client src/ydlidar_client.cpp)
ament_target_dependencies(ydlidar_client
  "rclcpp"
  "sensor_msgs"
  "visualization_msgs"
  "geometry_msgs"
)


install(TARGETS
  ydlidar_node ydlidar_client
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch params startup
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
#######################################################################
endif()
