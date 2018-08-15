# Prepare
swarmio_build_component_start(protobuf "3.6.0.1")

# Find Git executable
find_package(Git REQUIRED)

# Build protobuf libraries for target architecture
ExternalProject_Add(protobuf
	GIT_REPOSITORY "https://github.com/google/protobuf.git"
	GIT_TAG "48cb18e5c419ddd23d9badcfe4e9df7bde1979b2"
	PREFIX "protobuf-${SWARMIO_TARGET_ARCHITECTURE}"
	BUILD_COMMAND ${SWARMIO_TARGET_BUILD_COMMAND}
	INSTALL_COMMAND ${SWARMIO_TARGET_INSTALL_COMMAND}
	INSTALL_DIR ${PROTOBUF_ROOT}
	LIST_SEPARATOR "|"
	CONFIGURE_COMMAND ${CMAKE_COMMAND} "<SOURCE_DIR>/cmake"
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
	GIT_TAG "48cb18e5c419ddd23d9badcfe4e9df7bde1979b2"
	PREFIX "protobuf-host-${SWARMIO_HOST_ARCHITECTURE}"
	BUILD_COMMAND ${SWARMIO_HOST_BUILD_COMMAND}
	INSTALL_COMMAND ${SWARMIO_HOST_INSTALL_COMMAND}
	INSTALL_DIR "protobuf-host-${SWARMIO_HOST_ARCHITECTURE}-output"
	LIST_SEPARATOR "|"
	CONFIGURE_COMMAND ${CMAKE_COMMAND} "<SOURCE_DIR>/cmake"
		"-Dprotobuf_BUILD_TESTS=OFF"
		"-Dprotobuf_BUILD_EXAMPLES=OFF"
		${SWARMIO_SUBPROJECT_HOST_ARGS}
)

# Get protoc location
ExternalProject_Get_Property(protobuf-host INSTALL_DIR)
set(TOOL_PROTOC "${INSTALL_DIR}/bin/protoc")

