cmake_minimum_required(VERSION 3.0.2)
project(rtde_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  cartesian_control_msgs
  sensor_msgs
  tf2_msgs
  tf2_ros
)


## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)




###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES rtde_driver
#  CATKIN_DEPENDS roscpp 
#		 std_msgs
#		 geometry_msgs
#  		 cartesian_control_msgs
#  		 sensor_msgs
#  		 tf2_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/rtde_driver.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

 add_executable(telejoy src/telejoy.cpp
			src/get_urcontrol_version.cpp
			src/dashboard_client.cpp
			src/data_package.cpp
			src/default_log_handler.cpp
			src/log.cpp
			src/rtde_client.cpp
			src/rtde_writer.cpp
			src/rtde_package.cpp
			src/request_protocol_version.cpp
			src/tcp_socket.cpp
			src/text_message.cpp
			src/control_package_pause.cpp
			src/control_package_setup_outputs.cpp
			src/control_package_setup_inputs.cpp
			src/control_package_start.cpp

			)
 target_link_libraries(telejoy ${catkin_LIBRARIES})


