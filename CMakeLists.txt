cmake_minimum_required(VERSION 2.8.3)
project(aist_lenti_mark)

add_compile_options(-std=c++11)
add_compile_options(-D _CODE)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    rospy
    cv_bridge
    message_generation
    tf2_ros
    tf2_geometry_msgs
    shape_msgs
    visualization_msgs
    ddynamic_reconfigure
    rviz
)

find_package(OpenCV)

set(CMAKE_AUTOMOC ON)
find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED
  Core
  Widgets
)
set(QT_LIBRARIES Qt5::Widgets)
add_definitions(-DQT_NO_KEYWORDS)

set(LEAG_INCLUDE_DIRS "/usr/local/include/leag/")
set(LEAG_LIBS "/usr/local/lib/leag/libLentiMarkTracker.so")

add_message_files(
  FILES
    Marker.msg
    Markers.msg
    TexturedMeshStamped.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
    shape_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    message_runtime
    pluginlib
    nodelet
    tf2_ros
    tf2_geometry_msgs
    shape_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${LEAG_INCLUDE_DIRS}
  ${rviz_INCLUDE_DIRS}
)

message("-- LEAG_LIBS: ${LEAG_LIBS}")
message("-- OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}.cpp
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${LEAG_LIBS}
)

add_library(image_projector
  src/image_projector.cpp
)

add_dependencies(image_projector
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(image_projector
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${LEAG_LIBS}
)

add_library(textured_mesh_display
  src/textured_mesh_display.cpp
)

add_dependencies(textured_mesh_display
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(textured_mesh_display
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${QT_LIBRARIES}
)

install(
  TARGETS
    ${PROJECT_NAME} image_projector textured_mesh_display
  ARCHIVE
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
