cmake_minimum_required(VERSION 2.8.3)
project(ycx_planner)

find_package(catkin REQUIRED
        COMPONENTS
        base_local_planner
        costmap_2d
        nav_core
        pluginlib
        roscpp
        tf2
        tf2_geometry_msgs
        tf2_ros
)

find_package(OpenCV REQUIRED)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)
add_definitions(${EIGEN3_DEFINITIONS})

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ycx_planner
        CATKIN_DEPENDS
        base_local_planner
        costmap_2d
        nav_core
        pluginlib
        roscpp
        tf2
        tf2_ros
)

add_library(ycx_planner src/ycx_planner.cpp)
add_dependencies(ycx_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ycx_planner
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
)

install(TARGETS ycx_planner
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        PATTERN ".svn" EXCLUDE
)

install(FILES bgp_plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


