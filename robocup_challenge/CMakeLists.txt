cmake_minimum_required(VERSION 2.8.3)
project(robocup_challenge)

find_package(catkin REQUIRED)

catkin_package(
)

install(PROGRAMS
  scripts/utils.py
  scripts/move.py
  scripts/utils_takeshi.py
  scripts/takeshi_smach_11.py
  scripts/takeshi_smach_go_get_it.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
