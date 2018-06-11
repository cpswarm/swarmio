# Set minimum required version and baseline policy
cmake_minimum_required(VERSION 3.5)

# Declare project
project(swarmio-simulator LANGUAGES CXX)

# Enable C++17
set(CMAKE_CXX_STANDARD 17)

# Add target
add_executable(swarmio-simulator
	"src/main.cpp"
	"src/swarmio/simulator/ExampleDevice.cpp"
)

# Add include directories
target_include_directories(swarmio-simulator
	PUBLIC "include"
)

# Find required packages
find_package(libswarmio REQUIRED)

# Link libraries
target_link_libraries(swarmio-simulator PRIVATE
	libswarmio
)

# Use GNUInstallDirs to determine directories
include(GNUInstallDirs)

# Install application
install(
	TARGETS swarmio-simulator
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
