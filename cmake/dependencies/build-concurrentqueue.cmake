# Prepare
swarmio_build_component_start(concurrentqueue "1.0.0-8f7e861")

# Build concurrentqueue
ExternalProject_Add(concurrentqueue
    GIT_REPOSITORY "https://github.com/cameron314/concurrentqueue.git"
    GIT_TAG "8f7e861dd9411a0bf77a6b9de83a47b3424fafba"
    PREFIX "concurrentqueue"
    INSTALL_DIR ${CONCURRENTQUEUE_ROOT}
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    INSTALL_COMMAND ${CMAKE_COMMAND} -E make_directory "<INSTALL_DIR>/include"
            COMMAND ${CMAKE_COMMAND} -E copy "<SOURCE_DIR>/blockingconcurrentqueue.h" "<SOURCE_DIR>/concurrentqueue.h" "<INSTALL_DIR>/include"
)

# Finish
swarmio_build_component_finish(concurrentqueue)