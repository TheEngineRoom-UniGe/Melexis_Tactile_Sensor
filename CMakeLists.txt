cmake_minimum_required(VERSION 3.0.2)
project(melexis)

find_package(
	catkin REQUIRED COMPONENTS
	rospy
	std_msgs
	message_generation
	sensor_msgs
	roslaunch)

roslaunch_add_file_check(launch)
add_message_files(DIRECTORY msg)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs

)


catkin_package(

#	CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########


include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)

