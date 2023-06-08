cmake_minimum_required(VERSION 3.0.2)
project(egm_joint_state_to_mqtt_controller)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Release)


find_package(catkin REQUIRED COMPONENTS
    roscpp
    realtime_tools
    pluginlib
    controller_interface
    drapebot_mqtt_client
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES egm_joint_state_to_mqtt_controller
  CATKIN_DEPENDS roscpp controller_interface drapebot_mqtt_client
)


##############
## Settings ##
##############
if(NOT DEFINED BUILD_SHARED_LIBS)
  set(BUILD_SHARED_LIBS ON)
endif()

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  message(STATUS "${PROJECT_NAME}: Defaulting build type to RelWithDebInfo")
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()


###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

if(WIN32)
  include_directories("C:\\ws\\mosquitto\\include\\")
  #include_directories("C:\\ws\\jsoncpp\\include\\json\\")
endif()


add_library(${PROJECT_NAME} src/egm_joint_state_to_mqtt_controller.cpp)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


if(WIN32)
  target_link_directories(${PROJECT_NAME} PUBLIC "C:\\ws\\mosquitto-build\\lib\\")
  #target_link_directories(${PROJECT_NAME} PUBLIC "C:\\ws\\jsoncpp-build\\build\\lib\\Release\\")
endif()

target_link_libraries(${PROJECT_NAME} 
  ${catkin_LIBRARIES}
  mosquitto
  #jsoncpp
)

# Install
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# Install library
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(FILES egm_joint_state_to_mqtt_controller_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
