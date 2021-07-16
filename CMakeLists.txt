cmake_minimum_required(VERSION 2.10.2)
project(coldtech_demos)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	geometry_msgs
	moveit_core
	moveit_visual_tools
	moveit_ros_planning
	moveit_ros_planning_interface
	moveit_ros_perception
	pluginlib
	geometric_shapes
	pcl_ros
	pcl_conversions
	rosbag
	tf2_ros
	tf2_eigen
	tf2_geometry_msgs
	#message_generation
	)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED 
	system 
	filesystem 
	date_time 
	thread
	)


## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

#set(COLDTECH_DEMOS_INCLUDE_DIRS
#	planning/include
#	)

catkin_package(
	INCLUDE_DIRS 
	#${COLDTECH_DEMOS_INCLUDE_DIRS}
		#LIBRARIES river_ros
	CATKIN_DEPENDS 
		moveit_core
		moveit_visual_tools
		moveit_ros_planning_interface
		tf2_geometry_msgs
		std_msgs
		geometry_msgs
	DEPENDS 
		EIGEN3
)
include_directories(
	#${COLDTECH_DEMOS_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIR}
	${EIGEN3_INCLUDE_DIRS}
)



add_subdirectory(planning)
