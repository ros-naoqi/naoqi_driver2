cmake_minimum_required(VERSION 3.5)
project(naoqi_driver)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(naoqi_bridge_msgs REQUIRED)
find_package(naoqi_libqi REQUIRED)
find_package(naoqi_libqicore REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost QUIET COMPONENTS chrono filesystem program_options regex system thread random)

set(
  CONVERTERS_SRC
  src/converters/audio.cpp
  src/converters/touch.cpp
  src/converters/camera.cpp
  src/converters/diagnostics.cpp
  src/converters/imu.cpp
  src/converters/info.cpp
  src/converters/joint_state.cpp
  src/converters/laser.cpp
  src/converters/memory_list.cpp
  src/converters/memory/bool.cpp
  src/converters/memory/int.cpp
  src/converters/memory/float.cpp
  src/converters/memory/string.cpp
  src/converters/sonar.cpp
  src/converters/log.cpp
  src/converters/odom.cpp
  )
set(
  TOOLS_SRC
  src/tools/robot_description.cpp
  src/tools/from_any_value.cpp
  src/tools/subscribe_almemory.cpp
  )

set(
  PUBLISHER_SRC
  src/publishers/camera.cpp
  src/publishers/joint_state.cpp
  src/publishers/log.cpp
  src/publishers/sonar.cpp
  )

set(
  SUBSCRIBER_SRC
  src/subscribers/teleop.cpp
  src/subscribers/moveto.cpp
  src/subscribers/speech.cpp
  )

set(
  SERVICES_SRC
  src/services/robot_config.cpp
  src/services/set_language.cpp
  src/services/get_language.cpp
  )

set(
  ACTIONS_SRC
  src/actions/listen.cpp
)

set(
  RECORDER_SRC
  src/recorder/camera.cpp
  src/recorder/diagnostics.cpp
  src/recorder/joint_state.cpp
  src/recorder/log.cpp
  src/recorder/sonar.cpp
  )

set(
  FAKE_NAOQI_SRC
  src/fake_naoqi/fake_naoqi.cpp
  src/fake_naoqi/fake_almotion.cpp
  src/fake_naoqi/fake_almemory.cpp
  src/fake_naoqi/fake_services.cpp
)

set(
  DRIVER_SRC
  src/naoqi_driver.cpp
  src/helpers/driver_helpers.cpp
  src/helpers/ros_helpers.cpp
  src/recorder/globalrecorder.cpp
  src/event/basic.hxx
  src/event/basic.hpp
  src/event/audio.cpp
  src/event/touch.cpp
)

message(STATUS "Defining LIBQI_VERSION=${naoqi_libqi_VERSION_MAJOR}${naoqi_libqi_VERSION_MINOR})")
add_definitions(-DLIBQI_VERSION=${naoqi_libqi_VERSION_MAJOR}${naoqi_libqi_VERSION_MINOR})

# Add pre-processor compile flag indicating ament
add_definitions( -DAMENT_BUILD )

include_directories(
  "include"
  ${naoqi_libqi_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(
  naoqi_driver
  SHARED
  ${DRIVER_SRC}
  ${CONVERTERS_SRC}
  ${PUBLISHER_SRC}
  ${SUBSCRIBER_SRC}
  ${SERVICES_SRC}
  ${ACTIONS_SRC}
  ${RECORDER_SRC}
  ${TOOLS_SRC}
  ${FAKE_NAOQI_SRC}
)

ament_target_dependencies(naoqi_driver
  rclcpp
  rclcpp_action
  cv_bridge
  diagnostic_msgs
  diagnostic_updater
  geometry_msgs
  image_transport
  ament_index_cpp
  kdl_parser
  naoqi_bridge_msgs
  naoqi_libqi
  naoqi_libqicore
  robot_state_publisher
  sensor_msgs
  std_msgs
  tf2_geometry_msgs
  tf2_msgs
  tf2_ros
  OpenCV
  Boost

)

install(TARGETS naoqi_driver DESTINATION lib/)

# create the binary of the bridge
add_executable(naoqi_driver_node src/external_registration.cpp)
target_link_libraries(naoqi_driver_node
    naoqi_driver
    ${rclcpp_LIBRARIES}
    ${naoqi_libqi_LIBRARIES}
    ${Boost_LIBRARIES}
)

install(TARGETS naoqi_driver_node DESTINATION lib/${PROJECT_NAME})

# install the urdf for runtime loading
install(DIRECTORY share DESTINATION share/${PROJECT_NAME})

# install the launch files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_export_libraries(naoqi_driver)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_test "test/fake_almotion_test.cpp")
  target_include_directories(${PROJECT_NAME}_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(${PROJECT_NAME}_test naoqi_driver)
endif()

ament_package()
