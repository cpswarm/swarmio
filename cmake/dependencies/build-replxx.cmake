# Prepare
swarmio_build_component_start(replxx "0.0.1")

# Build replxx
ExternalProject_Add(replxx 
    GIT_REPOSITORY "https://github.com/AmokHuginnsson/replxx.git"
    GIT_TAG "3cb884e3fb4b1a28efeb716fac75f77eecc7ea3d"
    PREFIX "replxx-${SWARMIO_TARGET_ARCHITECTURE}"
    BUILD_COMMAND ${SWARMIO_TARGET_BUILD_COMMAND}
    INSTALL_COMMAND ${SWARMIO_TARGET_INSTALL_COMMAND}
    INSTALL_DIR ${REPLXX_ROOT}
    LIST_SEPARATOR "|"
    CMAKE_ARGS 
        ${REPLXX_CMAKE_ARGS}
)

# Finish
swarmio_build_component_finish(replxx)