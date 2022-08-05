cmake_minimum_required(VERSION 2.8.3)
project(aist_lenti_mark)

add_compile_options(-std=c++11)
add_compile_options(-D _CODE)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    cv_bridge
    image_transport
    std_msgs
    message_generation
    geometry_msgs
    tf
)

find_package(OpenCV)

set(LEAG_INCLUDE_DIRS "/usr/local/include/leag/")
set(LEAG_LIBS "/usr/local/lib/leag/libLentiMarkTracker.so")

add_message_files(
  FILES
    marker.msg
    markers.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    message_runtime
    geometry_msgs
    pluginlib
    nodelet
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${LEAG_INCLUDE_DIRS}
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

install(
  TARGETS
    ${PROJECT_NAME}
  ARCHIVE
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
