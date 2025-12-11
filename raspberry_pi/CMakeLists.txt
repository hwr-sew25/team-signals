cmake_minimum_required(VERSION 3.0.2)
project(signal_project)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package()

catkin_python_setup()

# Install Python modules
install(DIRECTORY signal_project/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/signal_project
)

# Install executable scripts
install(PROGRAMS
  signal_project/nodes/test_state_machine.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

