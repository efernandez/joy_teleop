cmake_minimum_required(VERSION 2.8.3)
project(joy_teleop)

find_package(catkin REQUIRED)

catkin_package()

install(PROGRAMS scripts/joy_teleop.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
