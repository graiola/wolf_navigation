cmake_minimum_required(VERSION 3.0.2)
project(wolf_navigation_utils)

find_package(PkgConfig)
pkg_check_modules(EIGEN eigen3)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    tf2
    tf2_ros
    tf2_eigen
    rt_gui
    message_filters
    )

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES wolf_navigation_utils
    )

include_directories(
  include
  ${EIGEN_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# odom publisher node
add_executable(odom_publisher_node src/odom_publisher.cpp src/estimators.cpp src/odom_publisher_node.cpp)
target_link_libraries(odom_publisher_node ${catkin_LIBRARIES})

# utils library
add_library(wolf_navigation_utils src/map_saver.cpp)
target_link_libraries(wolf_navigation_utils ${catkin_LIBRARIES})

install(TARGETS odom_publisher_node wolf_navigation_utils
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY maps/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/maps
  PATTERN ".svn" EXCLUDE)
