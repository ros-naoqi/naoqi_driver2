cmake_minimum_required(VERSION 3.5)
project(naoqi_driver)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
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
find_package(orocos_kdl REQUIRED)
find_package(Boost QUIET COMPONENTS chrono filesystem program_options regex system thread random)

# define source files that can be used by qibuild or catkin
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
  )

set(
  PUBLISHER_SRC
  src/publishers/camera.cpp
  src/publishers/info.cpp
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
  RECORDER_SRC
  src/recorder/camera.cpp
  src/recorder/diagnostics.cpp
  src/recorder/joint_state.cpp
  src/recorder/log.cpp
  src/recorder/sonar.cpp
  )

set(
  DRIVER_SRC
  src/naoqi_driver.cpp
  src/helpers/driver_helpers.cpp
  src/recorder/globalrecorder.cpp
  src/event/basic.hxx
  src/event/basic.hpp
  src/event/audio.cpp
  src/event/touch.cpp
)

add_definitions(-DLIBQI_VERSION=${naoqi_libqi_VERSION_MAJOR}${naoqi_libqi_VERSION_MINOR})

include_directories(include 
  ${naoqi_libqi_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS})

# create the different libraries
# add_library(
#   naoqi_driver_module
#   SHARED
#   src/autoload_registration.cpp
# )

# target_link_libraries(naoqi_driver_module ${naoqi_libqi_LIBRARIES})
# ament_target_dependencies(naoqi_driver_module
#     naoqi_libqi)

# install(TARGETS naoqi_driver_module DESTINATION lib/${PROJECT_NAME})

add_library(
  naoqi_driver
  SHARED
  ${DRIVER_SRC}
  ${CONVERTERS_SRC}
  ${PUBLISHER_SRC}
  ${SUBSCRIBER_SRC}
  ${SERVICES_SRC}
  ${RECORDER_SRC}
  ${TOOLS_SRC}
)

ament_target_dependencies(naoqi_driver
  rclcpp
  cv_bridge
  diagnostic_msgs
  diagnostic_updater
  geometry_msgs
  image_transport
  kdl_parser
  naoqi_bridge_msgs
  naoqi_libqi
  naoqi_libqicore
  robot_state_publisher
  sensor_msgs
  tf2_geometry_msgs
  tf2_msgs
  tf2_ros
  OpenCV
  orocos_kdl
  Boost)

install(TARGETS naoqi_driver DESTINATION lib/)

# target_link_libraries(naoqi_driver_module
#   naoqi_driver
# )

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
# file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/share DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/)
# install(DIRECTORY share DESTINATION "${CATKIN_PACKAGE_SHARE_DESTINATION}")
install(DIRECTORY share DESTINATION share/${PROJECT_NAME})

# install the launch files
# install(DIRECTORY launch DESTINATION "${CATKIN_PACKAGE_SHARE_DESTINATION}")
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# make sure there is a file describing a naoqi module
# file(WRITE ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_SHARE_DESTINATION}/qi/module/naoqi_driver_module.mod "cpp")
# install(FILES ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_SHARE_DESTINATION}/qi/module/naoqi_driver_module.mod DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/qi/module/)
# file(WRITE share/${PROJECT_NAME}/qi/module/naoqi_driver_module.mod "cpp")

# ament_export_libraries(naoqi_driver_module naoqi_driver)
ament_export_libraries(naoqi_driver)
ament_package()