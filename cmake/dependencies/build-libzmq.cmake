# Prepare
swarmio_build_component_start(libzmq "4.3.1-bb4fb32")

# Build libzmq
ExternalProject_Add(libzmq
    GIT_REPOSITORY "https://github.com/zeromq/libzmq.git"
    GIT_TAG "bb4fb32925c6465fd0f8e8cc89e5347bc79bc4bf"
    PREFIX "libzmq-${SWARMIO_TARGET_ARCHITECTURE}"
    BUILD_COMMAND ${SWARMIO_TARGET_BUILD_COMMAND}
    INSTALL_COMMAND ${SWARMIO_TARGET_INSTALL_COMMAND}
    INSTALL_DIR ${LIBZMQ_ROOT}
    LIST_SEPARATOR "|"
    CMAKE_ARGS 
        "-DWITH_PERF_TOOL=OFF" 
        "-DZMQ_BUILD_TESTS=OFF" 
        ${LIBZMQ_CMAKE_ARGS}
)

# Finish
swarmio_build_component_finish(libzmq)