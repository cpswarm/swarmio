# Set minimum required version and baseline policy
cmake_minimum_required(VERSION 3.5)

# Declare project
project(swarmio-tool LANGUAGES CXX)

# Enable C++17
set(CMAKE_CXX_STANDARD 17)

# Add target
add_executable(swarmio-tool
	"src/main.cpp"
	"src/swarmio/tool/Loop.cpp"
	"src/swarmio/tool/Command.cpp"
)

# Add include directories
target_include_directories(swarmio-tool PUBLIC 
	"include"
)

# Find required packages
find_package(libswarmio REQUIRED)
find_package(replxx REQUIRED)

# Link libraries
target_link_libraries(swarmio-tool PRIVATE
	libswarmio
	${REPLXX_LIBRARIES}
)

# Add library include paths
target_include_directories(swarmio-tool PRIVATE
	${REPLXX_INCLUDE_DIRS}
)

# Use GNUInstallDirs to determine directories
include(GNUInstallDirs)

# Install application
install(
	TARGETS swarmio-tool
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
