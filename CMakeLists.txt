cmake_minimum_required(VERSION 3.5)
project(mrs_uav_px4_api)

set(CATKIN_DEPENDENCIES
  cmake_modules
  geometry_msgs
  mavros
  mavros_msgs
  mrs_lib
  mrs_errorgraph
  mrs_msgs
  mrs_modules_msgs
  mrs_uav_hw_api
  nav_msgs
  pluginlib
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  )

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDENCIES}
  )

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(LIBRARIES
  MrsUavPx4Api_Api
  )

catkin_package(
  CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
  LIBRARIES ${LIBRARIES}
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
  )

# Plugin

add_library(MrsUavPx4Api_Api
  src/api.cpp
  )

add_dependencies(MrsUavPx4Api_Api
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(MrsUavPx4Api_Api
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

install(FILES plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
