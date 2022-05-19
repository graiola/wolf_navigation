cmake_minimum_required(VERSION 3.0.2)
project(wolf_waypoints)

find_package(PkgConfig)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    move_base
    actionlib
    rt_gui
    interactive_markers
    )

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(waypoints_node src/waypoints_node.cpp src/waypoints.cpp src/waypoint.cpp)
target_link_libraries(waypoints_node ${catkin_LIBRARIES})

install(TARGETS waypoints_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY media/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/media)
