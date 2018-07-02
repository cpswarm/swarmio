# Prepare
swarmio_build_component_start(libconfig "1.7.2-04a3674")

# Build libconfig
ExternalProject_Add(libconfig
    GIT_REPOSITORY "https://github.com/hyperrealm/libconfig.git"
    GIT_TAG "04a3674f8fa3f009444c8173259a3ea57ef6c35d"
    PREFIX "libconfig-${SWARMIO_TARGET_ARCHITECTURE}"
    BUILD_COMMAND ${SWARMIO_TARGET_BUILD_COMMAND}
    INSTALL_COMMAND ${SWARMIO_TARGET_INSTALL_COMMAND}
    INSTALL_DIR ${LIBCONFIG_ROOT}
    LIST_SEPARATOR "|"
    CMAKE_ARGS 
        "-DBUILD_TESTS=OFF"
        "-DBUILD_EXAMPLES=OFF"
        "-DBUILD_SHARED_LIBS=ON"
        ${LIBCONFIG_CMAKE_ARGS}
)

# Finish
swarmio_build_component_finish(libconfig)