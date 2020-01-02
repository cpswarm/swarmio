# Prepare
swarmio_build_component_start(sodium "1.0.18")
if("${SWARMIO_TARGET_ARCHITECTURE}" STREQUAL "arm64")
    set(HOST "--host=aarch64-linux-gnu")
elseif("${SWARMIO_TARGET_ARCHITECTURE}" STREQUAL "armhf")
    set(HOST "--host=arm-linux-gnueabihf")
else()
    set(HOST "")
endif()

# Build sodium
ExternalProject_Add(sodium 
    GIT_REPOSITORY "https://github.com/jedisct1/libsodium.git"
    GIT_TAG "940ef42797baa0278df6b7fd9e67c7590f87744b"
    PREFIX "sodium-${SWARMIO_TARGET_ARCHITECTURE}"
    INSTALL_DIR ${SODIUM_ROOT}
    CONFIGURE_COMMAND <SOURCE_DIR>/configure ${HOST} --prefix=<INSTALL_DIR>
    BUILD_COMMAND make
    INSTALL_COMMAND make install
    CMAKE_ARGS  
        ${SODIUM_CMAKE_ARGS}
)

# Finish
swarmio_build_component_finish(sodium)
