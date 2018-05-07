# Declare project
project(swarmio-tool LANGUAGES CXX)

# Add target
add_executable(swarmio-tool
	"src/main.cpp"
	"src/swarmio/tool/Loop.cpp"
	"src/swarmio/tool/Command.cpp"
)

# Add include directories
target_include_directories(swarmio-tool
	PUBLIC "include"
)

# Link libraries
target_link_libraries(swarmio-tool
	libswarmio
	${G3LOG_LIBRARIES}
	${REPLXX_LIBRARIES}
)

# Install application
install(TARGETS swarmio-tool
	EXPORT "libswarmio-targets"
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)