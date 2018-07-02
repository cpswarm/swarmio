# Prepare
swarmio_build_component_start(protobuf "3.6.0.1")

# Build protobuf libraries for target architecture
ExternalProject_Add(protobuf
	GIT_REPOSITORY "https://github.com/google/protobuf.git"
	GIT_TAG "ce044817c7ba0aea27c3fd8e496635d94d20a755"
	SOURCE_SUBDIR "cmake"
	PREFIX "protobuf-${SWARMIO_TARGET_ARCHITECTURE}"
	BUILD_COMMAND ${SWARMIO_TARGET_BUILD_COMMAND}
	INSTALL_COMMAND ${SWARMIO_TARGET_INSTALL_COMMAND}
	INSTALL_DIR ${PROTOBUF_ROOT}
	LIST_SEPARATOR "|"
	CMAKE_ARGS  
		"-Dprotobuf_BUILD_TESTS=OFF"
		"-Dprotobuf_BUILD_EXAMPLES=OFF"
		"-Dprotobuf_BUILD_SHARED_LIBS=ON"
		"-Dprotobuf_BUILD_PROTOC_BINARIES=OFF"
		${PROTOBUF_CMAKE_ARGS}
)

# Finish
swarmio_build_component_finish(protobuf)

# Build protobuf binaries for host architecture
ExternalProject_Add(protobuf-host
	GIT_REPOSITORY "https://github.com/google/protobuf.git"
	GIT_TAG "ce044817c7ba0aea27c3fd8e496635d94d20a755"
	SOURCE_SUBDIR "cmake"
	PREFIX "protobuf-host-${SWARMIO_HOST_ARCHITECTURE}"
	BUILD_COMMAND ${SWARMIO_HOST_BUILD_COMMAND}
	INSTALL_COMMAND ${SWARMIO_HOST_INSTALL_COMMAND}
	INSTALL_DIR "protobuf-host-${SWARMIO_HOST_ARCHITECTURE}-output"
	LIST_SEPARATOR "|"
	CMAKE_ARGS  
		"-Dprotobuf_BUILD_TESTS=OFF"
		"-Dprotobuf_BUILD_EXAMPLES=OFF"
		${SWARMIO_SUBPROJECT_HOST_ARGS}
)

# On UNIX-like systems, the current directory is not on PATH by default
if (UNIX)

	# Check for the presence of sed
	find_program(TOOL_SED "sed")
	if (TOOL_SED STREQUAL "TOOL_SED_NOT_FOUND")
		message(FATAL_ERROR "It seems that sed is not available on your system, install it before trying to build protobuf-host.")
	endif()

	# Patch protobuf to run js_embed from the correct location
	ExternalProject_Add_Step(protobuf-host patch-js_embed
		DEPENDEES update
		DEPENDERS configure
		ALWAYS true
		COMMAND ${TOOL_SED} -i -e "s@COMMAND js_embed@COMMAND \${CMAKE_CURRENT_BINARY_DIR}/js_embed@g" "<SOURCE_DIR>/cmake/libprotoc.cmake"
	)

endif()

# Get protoc location
ExternalProject_Get_Property(protobuf-host INSTALL_DIR)
set(TOOL_PROTOC "${INSTALL_DIR}/bin/protoc")

