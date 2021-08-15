################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(area_model)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS rospy)

################################################################################
# Setup for python modules and scripts
################################################################################
catkin_python_setup()

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  CATKIN_DEPENDS rospy
)

################################################################################
# Build
################################################################################

################################################################################
# Install
################################################################################
catkin_install_python(PROGRAMS
  src/turtlebot3_dqn_stage_1
  src/turtlebot3_dqn_stage_2
  src/turtlebot3_dqn_stage_3
  src/turtlebot3_dqn_stage_4
  src/result_graph
  src/action_graph
  src/moving_obstacle
  src/combination_obstacle_1
  src/combination_obstacle_2
  src/environment_stage_1.py
  src/environment_stage_2.py
  src/environment_stage_3.py
  src/environment_stage_4.py
  src/respawnGoal.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch save_model
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################
