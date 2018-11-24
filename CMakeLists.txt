cmake_minimum_required(VERSION 2.8.3)
project(golf_new_control)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        rosbag
        roscpp
        rospy
        std_msgs
        tf
        tf_conversions
        nodelet
	ramlab_msgs
        )

find_package(OpenCV REQUIRED)

catkin_package(
        LIBRARIES nodelet_joy_parser
        CATKIN_DEPENDS rosbag roscpp rospy std_msgs tf tf_conversions ramlab_msgs
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

link_directories(
        lib
)

#add_executable(teleop_node src/teleop_node.cpp)

add_executable(debug src/param_debug.cpp)

add_library(canbridge src/can_bridge.cpp)
add_library(encoder src/encoder_velocity_to_odom.cpp)
add_library(nodelet_joy_parser src/joy_parser.cpp)
add_library(nodelet_cmd_vel_mux src/cmd_vel_mux.cpp)
add_library(nodelet_can_parser src/can_parser.cpp)
add_library(nodelet_golf_can_parser src/golf_can_parser.cpp src/encoder_velocity_to_odom.cpp)

#add_executable(byd_vehicle_state_node src/can_parser.cpp src/can_bridge.cpp)
#add_executable(byd_vehicle_control_node src/byd_vehicle_control_node.cpp src/vel_pid.cpp include/vel_pid.h)

#target_link_libraries(teleop_node
#  ${catkin_LIBRARIES}
#)

target_link_libraries(canbridge
        ${catkin_LIBRARIES}
        controlcan
        )

target_link_libraries(nodelet_joy_parser
        ${catkin_LIBRARIES}
        )

target_link_libraries(nodelet_cmd_vel_mux
        ${catkin_LIBRARIES}
        )

target_link_libraries(nodelet_golf_can_parser
		${catkin_LIBRARIES}
		canbridge
		)

target_link_libraries(nodelet_can_parser
        ${catkin_LIBRARIES}
        canbridge
        encoder
        )
target_link_libraries(debug
                        ${catkin_LIBRARIES})
# install( TARGETS
#     costmap_2d_markers
#     costmap_2d_cloud
#     costmap_2d_node
#     DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

install(TARGETS
	canbridge
	nodelet_joy_parser 
	nodelet_cmd_vel_mux
	nodelet_can_parser
	nodelet_golf_can_parser
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
	FILES 
	can_parser.xml cmd_vel_mux.xml joy_parser.xml golf_can_parser.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
# install(
# 	DIRECTORY 
# 	include/ 
# 	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )
install(
	DIRECTORY 
	launch/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(
	DIRECTORY 
	lib/
	DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
