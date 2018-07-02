# Prepare
swarmio_build_component_start(zyre "2.0.0" libzmq czmq)

# Build zyre
ExternalProject_Add(zyre 
    GIT_REPOSITORY "https://github.com/zeromq/zyre.git"
    GIT_TAG "ab263dac21250fc85fa8c7239973109eca2251a7"
    PREFIX "zyre-${SWARMIO_TARGET_ARCHITECTURE}"
    BUILD_COMMAND ${SWARMIO_TARGET_BUILD_COMMAND}
    INSTALL_COMMAND ${SWARMIO_TARGET_INSTALL_COMMAND}
    INSTALL_DIR ${ZYRE_ROOT}
    LIST_SEPARATOR "|"
    CMAKE_ARGS  
        ${ZYRE_CMAKE_ARGS}        
)

# Finish
swarmio_build_component_finish(zyre)