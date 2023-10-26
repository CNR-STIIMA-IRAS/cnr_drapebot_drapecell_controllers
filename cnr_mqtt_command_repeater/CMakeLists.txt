cmake_minimum_required(VERSION 3.0.2)
project(cnr_mqtt_command_repeater)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  drapebot_mqtt_client
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cnr_mqtt_command_repeater
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs drapebot_mqtt_client
#  DEPENDS system_lib
)

###########
## Build ##
###########
if(WIN32)
  include_directories("C:\\ws\\mosquitto\\include\\")
endif()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/cnr_mqtt_command_repeater_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} drapebot_mqtt_client)

if(WIN32)
  target_link_directories(${PROJECT_NAME}_node PUBLIC "C:\\ws\\mosquitto-build\\lib\\")
endif()

#############
## Install ##
#############
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# Install library
install(TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

