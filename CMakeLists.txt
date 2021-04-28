cmake_minimum_required(VERSION 2.8.3)
project(hw1)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	tf2_ros 
	robotics_hw1 
	dynamic_reconfigure 
	nav_msgs 
	geometry_msgs 
	message_filters 
	message_generation
)

add_service_files(
  FILES
  SetOdometry.srv
)

add_message_files(
	FILES
	Odom.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
)

generate_dynamic_reconfigure_options(cfg/parameters.cfg)

## Declare a catkin package
catkin_package( CATKIN_DEPENDS geometry_msgs message_filters message_runtime)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(filter_msg src/filter_msg.cpp)
target_link_libraries(filter_msg ${catkin_LIBRARIES})
add_dependencies(filter_msg ${catkin_EXPORTED_TARGETS})

add_executable(gearratio_approx src/gearratio_approx.cpp)
target_link_libraries(gearratio_approx ${catkin_LIBRARIES})
add_dependencies(gearratio_approx ${catkin_EXPORTED_TARGETS})

add_executable(chiparam_approx src/chiparam_approx.cpp)
target_link_libraries(chiparam_approx ${catkin_LIBRARIES})
add_dependencies(chiparam_approx ${catkin_EXPORTED_TARGETS})

add_executable(odometry src/odometry.cpp)
add_dependencies(odometry ${PROJECT_NAME}_gencfg)
add_dependencies(odometry ${catkin_EXPORTED_TARGETS})
target_link_libraries(odometry ${catkin_LIBRARIES})

add_executable(reset_od_client src/reset_od_client.cpp)
target_link_libraries(reset_od_client ${catkin_LIBRARIES})
add_dependencies(reset_od_client ${catkin_EXPORTED_TARGETS})

add_executable(set_od_client src/set_od_client.cpp)
target_link_libraries(set_od_client ${catkin_LIBRARIES})
add_dependencies(set_od_client ${catkin_EXPORTED_TARGETS})

add_executable(odometry_tf src/odometry_tf.cpp)
target_link_libraries(odometry_tf ${catkin_LIBRARIES})

