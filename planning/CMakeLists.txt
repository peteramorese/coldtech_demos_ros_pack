add_executable(planning_demo_node src/planning_demo_node.cpp)

add_subdirectory(include)
target_include_directories(planning_demo_node PUBLIC include)
link_directories(planning_demo_node PUBLIC include)

target_link_libraries(planning_demo_node 
	EdgeClass
	AstarClass
	StateSpaceClass
	StateClass
	ConditionClass
	TransitionSystemClass
	${catkin_LIBRARIES}
	${Boost_LIBRARIES}
	)



