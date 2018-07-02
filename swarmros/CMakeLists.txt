# Set minimum required version and baseline policy
cmake_minimum_required(VERSION 3.5)

# Set project name
project(swarmros CXX)

# Enable C++17
set(CMAKE_CXX_STANDARD 17)

# Find required packages
find_package(swarmio REQUIRED)
find_package(libconfig REQUIRED)
find_package(catkin REQUIRED COMPONENTS
	roscpp
	rosconsole
	std_msgs
	message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  	Event.msg
)

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
	CATKIN_DEPENDS 
		roscpp
		rosconsole
		std_msgs 
)

# This is kind of hacky, but for some reason catkin insists 
# on putting the system directory into the include directory
if (NOT "${SWARMIO_SYSROOT}" STREQUAL "")
	list(REMOVE_ITEM catkin_INCLUDE_DIRS "/usr/include")
endif()

###########
## Build ##
###########

# Add main executable target
add_executable(bridge
	src/main.cpp
	src/swarmros/bridge/DebugSink.cpp
	src/swarmros/bridge/Device.cpp
	src/swarmros/bridge/EventForwarder.cpp
	src/swarmros/bridge/ParameterTarget.cpp
	src/swarmros/bridge/TelemetrySource.cpp
)

# Link with swarmio
target_link_libraries(bridge
	swarmio
	${LIBCONFIG_LIBRARIES}
)

# Since catkin uses pkg-config without caring about
# the new sysroot, we have to check that the library
# paths returned are actually correct.
if ("${CMAKE_SYSROOT}" STREQUAL "")
	target_link_libraries(bridge ${catkin_LIBRARIES})
else()
	foreach (_lib ${catkin_LIBRARIES})
		string(FIND "${_lib}" "${CMAKE_SYSROOT}" _location)
		if (_location EQUAL 0)
			target_link_libraries(bridge "${_lib}")
		else()
			target_link_libraries(bridge "${CMAKE_SYSROOT}${_lib}")
		endif()
	endforeach()
endif()

# Add include directory
target_include_directories(bridge
	PRIVATE
		include
		${LIBCONFIG_INCLUDE_DIRS}
		${catkin_INCLUDE_DIRS}
)

# Depend on message generation
add_dependencies(bridge
	${swarmros_EXPORTED_TARGETS}
	${catkin_EXPORTED_TARGETS}
)

# Add config file reference macro
target_compile_definitions(bridge PRIVATE
	"SWARMROS_CONFIG_PATH=\"${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_ETC_DESTINATION}/swarmros.cfg\""
)

#############
## Install ##
#############

install(
	TARGETS bridge 
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
	DIRECTORY "include"
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.h"
)

install(
	FILES "${CMAKE_SOURCE_DIR}/resources/swarmros.cfg"
	DESTINATION ${CATKIN_PACKAGE_ETC_DESTINATION}
)
