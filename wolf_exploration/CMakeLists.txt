cmake_minimum_required(VERSION 3.1)
project(wolf_exploration)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  costmap_2d
  geometry_msgs
  map_msgs
  move_base_msgs
  nav_msgs
  roscpp
  std_msgs
  tf
  visualization_msgs
  costmap_converter
  wolf_navigation_utils
)

## Find OPTIONAL libraries
find_package(artifacts_mapping QUIET)
find_package(rt_gui QUIET)

if(${rt_gui_FOUND})
    message("Found rt_gui, creating interfaces for wolf_navigation.")
    add_definitions(-DRT_GUI)
else()
    message("rt_gui not found.")
endif()

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    actionlib
    actionlib_msgs
    costmap_2d
    geometry_msgs
    map_msgs
    move_base_msgs
    nav_msgs
    roscpp
    std_msgs
    tf
    visualization_msgs
    costmap_converter
    wolf_navigation_utils
)

###########
## Build ##
###########
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_library(exploration_tools
  src/costmap_client.cpp
  src/costmap_server.cpp
  src/frontier_search.cpp
  src/artifacts_search.cpp
  src/map_explorer.cpp
)
add_dependencies(exploration_tools ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(exploration_tools ${catkin_LIBRARIES})

add_executable(map_explorer_node src/map_explorer_node.cpp)
add_dependencies(map_explorer_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(map_explorer_node exploration_tools ${catkin_LIBRARIES})


if(${artifacts_mapping_FOUND})
    message("Found artifacts_mapping, compiling artifacts_search_node for wolf_navigation.")

    add_executable(artifacts_search_node src/artifacts_search_node.cpp)
    add_dependencies(artifacts_search_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_link_libraries(artifacts_search_node exploration_tools ${catkin_LIBRARIES})

    # install nodes
    install(TARGETS artifacts_search_node
      ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

else()
    message("artifacts_mapping not found.")
endif()

#############
## Install ##
#############

# install nodes
install(TARGETS exploration_tools map_explorer_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install roslaunch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)

  add_executable(test_cluster src/test.cpp)
  add_dependencies(test_cluster ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(test_cluster exploration_tools ${catkin_LIBRARIES})

  # test all launch files
  roslaunch_add_file_check(launch)
endif()
