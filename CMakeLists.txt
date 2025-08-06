cmake_minimum_required(VERSION 3.0.2)
project(ankle_exoskeleton)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  dynamixel_sdk
  message_generation
  cmake_modules
  roslib
)

find_package(Eigen3 REQUIRED)

# Resolve system dependency on yaml-cpp, which apparently does not
# provide a CMake find_package() module.
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR
  NAMES yaml_cpp.h
  PATHS ${YAML_CPP_INCLUDE_DIRS}
)
find_library(YAML_CPP_LIBRARY
  NAMES YAML_CPP
  PATHS ${YAML_CPP_LIBRARY_DIRS}
)
link_directories(${YAML_CPP_LIBRARY_DIRS})

if(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")



## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   DynamixelStatus.msg
   DynamixelStatusList.msg
   DynamixelStatusPhysical.msg
   DynamixelStatusListPhysical.msg
   IMUData.msg
 )

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   DynamixelCommand.srv
   DynamixelCmdSimplified.srv
 )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )



###################################
## catkin specific configuration ##
###################################

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ankle_exoskeleton
   CATKIN_DEPENDS roscpp dynamixel_sdk message_runtime cmake_modules roslib
   DEPENDS EIGEN3
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)
   
add_library(dynamixel_workbench
  src/dynamixel_workbench/dynamixel_item.cpp
  src/dynamixel_workbench/dynamixel_driver.cpp
  src/dynamixel_workbench/dynamixel_tool.cpp
  src/dynamixel_workbench/dynamixel_workbench.cpp
)
add_dependencies(dynamixel_workbench ${catkin_EXPORTED_TARGETS})
target_link_libraries(dynamixel_workbench ${catkin_LIBRARIES})

add_executable(dynamixel_controllers src/nodes/dynamixel_controllers.cpp)
target_link_libraries(dynamixel_controllers ${catkin_LIBRARIES} dynamixel_workbench ${Eigen3_LIBRARIES} ${YAML_CPP_LIBRARIES})

################################################################################
# Install
################################################################################
install(TARGETS dynamixel_workbench dynamixel_controllers
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)



