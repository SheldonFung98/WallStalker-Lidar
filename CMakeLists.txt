cmake_minimum_required(VERSION 3.0.2)

project(laser_stickwall)
set(CMAKE_CXX_STANDARD 11)

find_package(
  catkin REQUIRED
  roscpp
  genmsg
  sensor_msgs
  pcl_conversions 
  pcl_ros
  tf
  laser_geometry
  cv_bridge
  nav_msgs
)

generate_messages(DEPENDENCIES 
sensor_msgs
)
catkin_package()

find_package(OpenCV 3 REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

file(GLOB 
SRC_FILES 
src/main.cpp
src/wallDetect.cpp
)

add_executable(${PROJECT_NAME}_node 
  ${SRC_FILES}
)
target_link_libraries(${PROJECT_NAME}_node 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
add_dependencies(
  ${PROJECT_NAME}_node 
  ${PROJECT_NAME}_generate_messages_cpp
)

install( TARGETS ${PROJECT_NAME}_node 
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)


#############
## Cpack ##
#############

string(TIMESTAMP COMPILE_TIME %Y%m%d-%H%M%S)
set(build_time    ${COMPILE_TIME})

set(CPACK_GENERATOR "DEB")

#Creating UNIX structured package
set(CPACK_CMAKE_GENERATOR "Unix Makefiles")
set(CPACK_PACKAGE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/")
set(CPACK_PACKAGE_VERSION 0.0.1-${build_time})

#Set directory to /../ to avoid /share part of ROS_PACKAGE_PATH
set(CPACK_PACKAGING_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX})
set(CPACK_INSTALL_CMAKE_PROJECTS "${CMAKE_CURRENT_BINARY_DIR};${PROJECT_NAME};ALL;/./")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY
    "${PROJECT_NAME} - ${PROJECT_DESCRIPTION}")
set(CPACK_PACKAGE_NAME "${PROJECT_NAME}")
set(CPACK_PACKAGE_VENDOR "RCS")

#Now we can simply create the dependencies using almost the same package names as they declared in ROS repository
#set(CPACK_DEBIAN_PACKAGE_DEPENDS "ros-kinetic-roscpp, ros-kinetic-std-msgs, ros-kinetic-message-runtime, ros-kinetic-sensor-msgs, ros-kinetic-libmodbus")

#IMPORTANT: set this to keep on auto-introspection for system dependencies like libc++6
#set (CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON)

#Keep Debian package structure untouched
set(CPACK_DEBIAN_PACKAGE_CONTROL_STRICT_PERMISSION TRUE)

#Standard naming schema for Debian-based distros
set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION}~${CMAKE_SYSTEM_NAME}_${CMAKE_SYSTEM_PROCESSOR}")
set(CPACK_PACKAGE_VENDOR "Technical University")
set(CPACK_PACKAGE_CONTACT "<your@email.com>")

#Turn the process on    
include(CPack)
