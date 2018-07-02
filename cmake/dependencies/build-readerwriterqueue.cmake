# Prepare
swarmio_build_component_start(readerwriterqueue "1.0.0-07e22ec")

# Build readerwriterqueue
ExternalProject_Add(readerwriterqueue
    GIT_REPOSITORY "https://github.com/cameron314/readerwriterqueue.git"
    GIT_TAG "07e22ecdf90501df89ead679bb8294a0b7c80c24"
    PREFIX "readerwriterqueue"
    INSTALL_DIR ${READERWRITERQUEUE_ROOT}
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    INSTALL_COMMAND ${CMAKE_COMMAND} -E make_directory "<INSTALL_DIR>/include"
            COMMAND ${CMAKE_COMMAND} -E copy "<SOURCE_DIR>/readerwriterqueue.h" "<SOURCE_DIR>/atomicops.h" "<INSTALL_DIR>/include"
)

# Finish
swarmio_build_component_finish(readerwriterqueue)