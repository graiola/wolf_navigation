cmake_minimum_required(VERSION 3.0.2)
project(wolf_navigation)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    tf
    )

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# odom publisher node
add_executable(odom_publisher_node src/odom_publisher_node.cpp)

target_link_libraries(odom_publisher_node
${catkin_LIBRARIES}
)

catkin_package()

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY params/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/params
  PATTERN ".svn" EXCLUDE)
