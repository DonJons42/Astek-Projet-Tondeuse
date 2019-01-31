cmake_minimum_required(VERSION 2.8.3)
project(teleop_twist_keyboard)

find_package(catkin REQUIRED)

catkin_package()

include_directories(
${catkin_INCLUDE_DIRS})

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
   teleop_twist_keyboard.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

