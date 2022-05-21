cmake_minimum_required(VERSION 2.8.3)
project(lenti_mark)
add_compile_options(-std=c++11)

add_compile_options(-D _CODE)

find_package(catkin REQUIRED COMPONENTS
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
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${LEAG_INCLUDE_DIRS}
  "include/lenti_mark/"
)

message("-- LEAG_LIBS: ${LEAG_LIBS}")
message("-- OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")

add_executable(lenti_mark src/lenti_mark.cpp)
add_dependencies(lenti_mark ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(lenti_mark ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${LEAG_LIBS})

