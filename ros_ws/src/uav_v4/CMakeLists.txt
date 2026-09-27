cmake_minimum_required(VERSION 2.8.3)

project(uav_v4)

find_package(catkin REQUIRED COMPONENTS
	diagnostic_msgs
	geometry_msgs
	roscpp
	std_msgs
)
find_package(gazebo REQUIRED)

catkin_package(
	LIBRARIES mode_aware_planar_move_plugin
	CATKIN_DEPENDS diagnostic_msgs geometry_msgs roscpp std_msgs
)

include_directories(
	${catkin_INCLUDE_DIRS}
	${GAZEBO_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(mode_aware_planar_move_plugin
	src/mode_aware_planar_move_plugin.cpp
)
target_compile_features(mode_aware_planar_move_plugin PRIVATE cxx_std_14)
target_link_libraries(mode_aware_planar_move_plugin
	${catkin_LIBRARIES}
	${GAZEBO_LIBRARIES}
)

# Phase E1a diagnostic only. This world plugin observes the planar plugin
# before/after WorldUpdateBegin and after ODE at WorldUpdateEnd.
add_library(ground_contact_diagnostic_plugin
	src/ground_contact_diagnostic_plugin.cpp
)
target_compile_features(ground_contact_diagnostic_plugin PRIVATE cxx_std_14)
target_link_libraries(ground_contact_diagnostic_plugin
	${catkin_LIBRARIES}
	${GAZEBO_LIBRARIES}
)

add_library(takeoff_clearance_plugin
	src/takeoff_clearance_plugin.cpp
)
target_compile_features(takeoff_clearance_plugin PRIVATE cxx_std_14)
target_link_libraries(takeoff_clearance_plugin
	${catkin_LIBRARIES}
	${GAZEBO_LIBRARIES}
)

find_package(roslaunch)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

install(TARGETS mode_aware_planar_move_plugin ground_contact_diagnostic_plugin
	takeoff_clearance_plugin
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
