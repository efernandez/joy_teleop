cmake_minimum_required(VERSION 2.8.3)
project(marker_teleop)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
  interactive_markers
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES marker_teleop
#  CATKIN_DEPENDS interactive_markers roscpp visualization_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(marker_teleop_node src/marker_teleop_node.cpp)
target_link_libraries(marker_teleop_node ${catkin_LIBRARIES})

install(TARGETS marker_teleop_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir launch)
  install(DIRECTORY ${dir}/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach()
