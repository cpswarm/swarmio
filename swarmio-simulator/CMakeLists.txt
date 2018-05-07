# Declare project
project(swarmio-simulator LANGUAGES CXX)

# Add target
add_executable(swarmio-simulator
	"src/main.cpp"
	"src/swarmio/simulator/ExampleDevice.cpp"
)

# Add include directories
target_include_directories(swarmio-simulator
	PUBLIC "include"
)

# Link libraries
target_link_libraries(swarmio-simulator
	libswarmio
	${G3LOG_LIBRARIES}
)

# Install application
install(TARGETS swarmio-simulator
	EXPORT "libswarmio-targets"
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)