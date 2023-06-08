cmake_minimum_required(VERSION 3.0.2)
project(drapebot_mqtt_client)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Release)


find_package(catkin REQUIRED COMPONENTS 
            roscpp
            cnr_mqtt_client)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES drapebot_mqtt_client
  CATKIN_DEPENDS roscpp cnr_mqtt_client
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories( include
 ${catkin_INCLUDE_DIRS}
)

if(WIN32)
  include_directories("C:\\ws\\mosquitto\\include\\")
  #include_directories("C:\\ws\\jsoncpp\\include\\json\\")
endif()

## Declare a C++ library
add_library(${PROJECT_NAME} 
  src/drapebot_mqtt_client.cpp
)

if(WIN32)
  target_link_directories(${PROJECT_NAME} PUBLIC "C:\\ws\\mosquitto-build\\lib\\")
  #target_link_directories(${PROJECT_NAME} PUBLIC "C:\\ws\\jsoncpp-build\\build\\lib\\Release\\")
endif()

target_link_libraries(${PROJECT_NAME} 
  ${catkin_LIBRARIES}
  mosquitto
)

################################################
# Declare a C++ executable
add_executable(${PROJECT_NAME}_node 
  src/drapebot_mqtt_client_node.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node 
  ${catkin_LIBRARIES}
  ${PROJECT_NAME} 
)
################################################



#############
## Install ##
#############
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(FILES
  # myfile1
  # myfile2
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)