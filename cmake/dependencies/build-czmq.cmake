# Prepare
swarmio_build_component_start(czmq "4.1.1" libzmq)

# Build czmq
ExternalProject_Add(czmq
    GIT_REPOSITORY "https://github.com/cpswarm/czmq.git"
    GIT_TAG "cbcb1a566624911fe0332897cf6f327d7820ef32"
    PREFIX "czmq-${SWARMIO_TARGET_ARCHITECTURE}"
    BUILD_COMMAND ${SWARMIO_TARGET_BUILD_COMMAND}
    INSTALL_COMMAND ${SWARMIO_TARGET_INSTALL_COMMAND}
    INSTALL_DIR ${CZMQ_ROOT}
    LIST_SEPARATOR "|"
    CMAKE_ARGS
        ${CZMQ_CMAKE_ARGS}
)

# Finish
swarmio_build_component_finish(czmq)
