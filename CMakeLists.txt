cmake_minimum_required(VERSION 2.8.3)

project(view)

find_package(catkin REQUIRED
	roscpp
	nav_msgs
	geometry_msgs
)

find_package(Eigen3)
find_package(OpenCV)

include_directories(${catkin_INCLUDE_DIRS}  ${EIGEN3_INCLUDE_DIR})

catkin_package()

add_executable(view_node  src/view_node.cpp  src/CameraPoseVisualization.cpp)
target_link_libraries( view_node ${catkin_LIBRARIES} ${OpenCV_LIBS})