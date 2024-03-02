cmake_minimum_required(VERSION 3.8)
project(chassis)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(x264 QUIET)

execute_process(COMMAND uname -m OUTPUT_VARIABLE MACHINES)
execute_process(COMMAND getconf LONG_BIT OUTPUT_VARIABLE MACHINES_BIT)
message(STATUS "Machine : ${MACHINES}")
message(STATUS "Machine Bits : ${MACHINES_BIT}")

# set(CMAKE_BUILD_TYPE "Debug")
if (${CMAKE_BUILD_TYPE} MATCHES "Debug")
  add_definitions(-DBACKTRACE_DEBUG)
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

set(USE_LIVE555 OFF)

if(USE_LIVE555)

set(LIVE555URL http://www.live555.com/liveMedia/public/live555-latest.tar.gz CACHE STRING "live555 url")
set(LIVE555CFLAGS -DBSD=1 -DSOCKLEN_T=socklen_t -D_FILE_OFFSET_BITS=64 -D_LARGEFILE_SOURCE=1 -DALLOW_RTSP_SERVER_PORT_REUSE=1 -DNO_OPENSSL=1 CACHE STRING "live555 CFGLAGS")

# openssl ?
find_package(OpenSSL QUIET)

# live555
find_package(libliveMedia)
find_package(libUsageEnvironment)
find_package(libBasicUsageEnvironment)
find_package(libgroupsock)
if (LIBLIVEMEDIA_FOUND AND LIBBASICUSAGEENVIRONMENT_FOUND AND LIBUSAGEENVIRONMENT_FOUND AND LIBGROUPSOCK_FOUND)
    message(STATUS "live555 loaded")
    set(LIVEINCLUDE ${LIBLIVEMEDIA_INCLUDE_DIR} ${LIBBASICUSAGEENVIRONMENT_INCLUDE_DIR} ${LIBUSAGEENVIRONMENT_INCLUDE_DIR} ${LIBGROUPSOCK_INCLUDE_DIR})
    SET(LIVE_LIBRARIES ${LIBLIVEMEDIA_LIBRARY} ${LIBUSAGEENVIRONMENT_LIBRARY} ${LIBBASICUSAGEENVIRONMENT_LIBRARY} ${LIBGROUPSOCK_LIBRARY}) 
    SET(CPACK_DEBIAN_PACKAGE_DEPENDS ${CPACK_DEBIAN_PACKAGE_DEPENDS}livemedia-utils,)
else()
    set(LIVE ${CMAKE_BINARY_DIR}/live)
    set(LIVEINCLUDE ${LIVE}/groupsock/include ${LIVE}/liveMedia/include ${LIVE}/UsageEnvironment/include ${LIVE}/BasicUsageEnvironment/include)
    if (NOT EXISTS ${LIVE})
        file (DOWNLOAD ${LIVE555URL} ${CMAKE_BINARY_DIR}/live555-latest.tar.gz )
        EXECUTE_PROCESS(COMMAND ${CMAKE_COMMAND} -E tar xvf ${CMAKE_BINARY_DIR}/live555-latest.tar.gz  RESULT_VARIABLE unpack_result)
        if(NOT unpack_result STREQUAL "0")
            message(FATAL_ERROR "Fetching and compiling live555 failed!")
        endif()
    endif()
    FILE(GLOB LIVESOURCE ${LIVE}/groupsock/*.c* ${LIVE}/liveMedia/*.c* ${LIVE}/UsageEnvironment/*.c* ${LIVE}/BasicUsageEnvironment/*.c*)
    if (NOT OpenSSL_FOUND)
        set(LIVE555CFLAGS ${LIVE555CFLAGS} -DNO_OPENSSL=1)
    endif()
    add_definitions(${LIVE555CFLAGS})
endif()

add_library (liblive555 STATIC ${LIVESOURCE})

add_definitions(-DUSE_LIVE555)

include_directories(
    ${LIVEINCLUDE}
    src/live_video/ringbuf
    src/live_video
)

file(GLOB live_srcs 
    src/live_video/*.cpp
)

else()
# 自定义的librtsp
include_directories(
  src/librtsp
)

file(GLOB live_srcs 
  src/librtsp/*.cpp
)

endif()

include_directories(
    include
    src/drivers # 需要SOC资源
    src/moto    # 我需要轮子
    src/remote  # 需要遥控器
    src/sensor  # 需要传感器
    src/motion  # 运动控制
    src/sensor/camera
    src/music
    # src/sensor/audio_source
    src/sensor/H264_camera
    src/utils
    src/lora
    src/sensor/mpu6050
    src/sensor/mpu9250
    src/sensor/imu_modules
)

file(GLOB main_srcs 
    src/*.cpp
    src/drivers/*.cpp
    src/sensor/*.cpp
    src/music/*.cpp
    # src/sensor/audio_source/*cpp
    src/sensor/camera/*.cpp
    src/sensor/H264_camera/*.cpp
    src/moto/*.cpp
    src/motion/*.cpp
    src/remote/*.cpp
    src/utils/*.cpp
    src/lora/*.cpp
    src/sensor/mpu6050/*.cpp
    src/sensor/mpu9250/*.cpp
    src/sensor/imu_modules/*.cpp
)

add_subdirectory(src/lib_json)

if(USE_ROS1_MIDDLE_PRICE)
######################## 使用ROS1 作为中间件 ###########################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
  sensor_msgs
  nav_msgs
)

# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   geometry_msgs
# )

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
)

add_executable(${PROJECT_NAME}_node ${main_srcs} ${live_srcs} ${LIVESOURCE})
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS}) 

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} pthread json x264 asound faac mpg123 ${LIBSPDLOG})

install(DIRECTORY ${PROJECT_NAME}_node
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
#######################################################################
else()
######################## 使用ROS2 作为中间件 ###########################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
    ${roscpp_INCLUDE_DIRS} 
    ${roslib_INCLUDE_DIRS} 
)

add_executable(${PROJECT_NAME}_node ${main_srcs} ${live_srcs} ${LIVESOURCE})
if (${MACHINES} MATCHES "aarch64")
# target_link_libraries(${PROJECT_NAME}_node pthread json x264 mali asound faac mpg123)
target_link_libraries(${PROJECT_NAME}_node pthread json x264 asound faac mpg123 ${LIBSPDLOG})
else()
target_link_libraries(${PROJECT_NAME}_node pthread json x264 asound faac mpg123 ${LIBSPDLOG})
endif()
ament_target_dependencies(${PROJECT_NAME}_node rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs)

install(DIRECTORY launch params
		DESTINATION share/${PROJECT_NAME}
)
  
install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
#######################################################################
endif()