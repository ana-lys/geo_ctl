cmake_minimum_required(VERSION 2.8.12)
project(geometric_controller)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  dynamic_reconfigure
  tf
  mavros
  quadrotor_msgs
  mavros_extras
  mavros_msgs
  mavlink
)

generate_dynamic_reconfigure_options(
  cfg/GeometricController.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES geometric_controller
  CATKIN_DEPENDS roscpp rospy std_msgs mavros_msgs geometry_msgs sensor_msgs tf quadrotor_msgs
)

#############
# LIBRARIES #
#############
include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/geometric_controller.cpp
)
add_library(geometric_loaded 
  src/geometric_loaded.cpp
)
add_library(geometric_recorder
  src/geometric_recorder.cpp
)

add_dependencies(geometric_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(geometric_loaded  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(geometric_recorder ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
############
# BINARIES #
############
add_executable(geometric_controller_node
  src/geometric_controller_node.cpp
)
add_executable(geometric_loaded_node
  src/geometric_loaded_node.cpp
)
add_executable(geometric_recorder_node
  src/geometric_recorder_node.cpp
)

add_executable(land
  src/land.cpp
)
add_executable(stat
  src/stat.cpp
)
add_executable(load
  src/load.cpp
)
add_dependencies(geometric_controller_node geometric_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(geometric_loaded_node geometric_loaded ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(geometric_recorder_node geometric_recorder ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(geometric_controller_node ${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(geometric_loaded_node geometric_loaded ${catkin_LIBRARIES})
target_link_libraries(geometric_recorder_node geometric_recorder ${catkin_LIBRARIES})

target_link_libraries(land ${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(stat ${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(load ${PROJECT_NAME} ${catkin_LIBRARIES})