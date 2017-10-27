cmake_minimum_required(VERSION 2.8.3)
project(friendly_faces)

add_definitions(-std=c++11)

find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
	roscpp
	cv_bridge
	image_transport
	sensor_msgs
	std_msgs
	message_generation
	geometry_msgs
)

catkin_package(
	INCLUDE_DIRS include/ ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}
	CATKIN_DEPENDS std_msgs
)

# BUILD

include_directories(include/ ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(friendly_faces_runner
	src/main.cpp
)
add_dependencies(friendly_faces_runner ${catkin_EXPORTED_TARGETS})
target_link_libraries(friendly_faces_runner ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
