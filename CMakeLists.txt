cmake_minimum_required(VERSION 3.5)
project(mrs_multirotor_simulator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

add_definitions(-Wall)
add_definitions(-Wextra)

set(CATKIN_DEPENDENCIES
  cmake_modules
  dynamic_reconfigure
  geometry_msgs
  mrs_lib
  mrs_msgs
  mrs_uav_hw_api
  nav_msgs
  nodelet
  roscpp
  rosgraph_msgs
  sensor_msgs
  std_msgs
  std_srvs
  )

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDENCIES}
  )

generate_dynamic_reconfigure_options(
  cfg/multirotor_simulator.cfg
  )

set(LIBRARIES
  MrsMultirotorSimulator_Simulator
  MrsMultirotorSimulator_SimulatorSystemRos
  MrsMultirotorSimulator_HwApiPlugin
  ${LIB_UEDS_CONNECTOR}
  )

catkin_package(
  LIBRARIES ${LIBRARIES}
  INCLUDE_DIRS include include/mrs_multirotor_simulator include/mrs_multirotor_simulator/uav_system include/mrs_multirotor_simulator/uav_system/controllers extern/ueds-connector/include/ueds-connector
  CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
  )

include_directories(
  include
  include/mrs_multirotor_simulator
  include/mrs_multirotor_simulator/uav_system
  include/mrs_multirotor_simulator/uav_system/controllers
  extern/ueds-connector/include
  ${catkin_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
  )

# udes-connector library
set(LIB_UEDS_CONNECTOR ueds-connector)
set(PATH_SRC extern/ueds-connector/src)
set(SOURCES ${PATH_SRC}/socket-client.cpp ${PATH_SRC}/drone-controller.cpp ${PATH_SRC}/game-mode-controller.cpp)

add_library(${LIB_UEDS_CONNECTOR} OBJECT ${SOURCES})
target_include_directories(${LIB_UEDS_CONNECTOR} PUBLIC extern/ueds-connector/include)

# add_dependencies(${LIB_UEDS_CONNECTOR} 
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
#   ${catkin_EXPORTED_TARGETS}
#   )

# target_link_libraries(${LIB_UEDS_CONNECTOR} 
#   ${catkin_LIBRARIES}
#   ${Eigen_LIBRARIES}
#   )

# ROS Wrapper for the Multirotor UAV System + controllers

add_library(MrsMultirotorSimulator_SimulatorSystemRos
  src/uav_system_ros.cpp
  )

add_dependencies(MrsMultirotorSimulator_SimulatorSystemRos
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(MrsMultirotorSimulator_SimulatorSystemRos
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

# Multirotor simulator
# - ROS node with the MRS Multirotor Simulator

add_library(MrsMultirotorSimulator_Simulator
  src/multirotor_simulator.cpp
  )

add_dependencies(MrsMultirotorSimulator_Simulator
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(MrsMultirotorSimulator_Simulator
  MrsMultirotorSimulator_SimulatorSystemRos
  ${LIB_UEDS_CONNECTOR}
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

# Hw API Plugin
# - pluginlib library for interfacing the Simulator with the MRS UAV System

add_library(MrsMultirotorSimulator_HwApiPlugin
  src/hw_api_plugin.cpp
  )

add_dependencies(MrsMultirotorSimulator_HwApiPlugin
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(MrsMultirotorSimulator_HwApiPlugin
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )


## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(TARGETS ${LIBRARIES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(DIRECTORY tmux/
  USE_SOURCE_PERMISSIONS
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/tmux
  )

install(DIRECTORY include/mrs_multirotor_simulator/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

install(DIRECTORY scripts/
  USE_SOURCE_PERMISSIONS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY scripts
  USE_SOURCE_PERMISSIONS
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(FILES nodelets.xml plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
