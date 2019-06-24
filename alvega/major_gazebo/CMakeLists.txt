cmake_minimum_required(VERSION 2.8.3)
project(major_gazebo)

find_package(catkin REQUIRED)

catkin_package()

#catkin_install_python(PROGRAMS scripts/ackermann_controller
#                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
