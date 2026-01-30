cmake_minimum_required(VERSION 3.0.2)
project(signal_project)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  smach
  smach_ros
)

catkin_package()

catkin_python_setup()

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install executable scripts
catkin_install_python(PROGRAMS
  signal_project/nodes/state_machine_node.py
  signal_project/nodes/test_state_machine.py
  signal_project/nodes/signal_controller_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
