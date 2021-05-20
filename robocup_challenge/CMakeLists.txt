cmake_minimum_required(VERSION 2.8.3)
project(robocup_challenge)

find_package(catkin REQUIRED)

catkin_package(
)

install(PROGRAMS
  #scripts/utils.py
  #scripts/move.py
  #scripts/utils_takeshi.py
  #scripts/takeshi_smach_task1.py
  pumas_navigation/act_pln/scripts/utils.py
  pumas_navigation/act_pln/scripts/utils_takeshi.py
  pumas_navigation/act_pln/scripts/takeshi_smach_go_get_it.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  #launch
  pumas_navigation/takeshi_smach_go_get_it/launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
