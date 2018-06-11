# System name
set(CMAKE_SYSTEM_NAME Linux)

# Tools
set(TOOLCHAIN_PREFIX aarch64-linux-gnu)
set(CMAKE_C_COMPILER ${TOOLCHAIN_PREFIX}-gcc)
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PREFIX}-g++)

# Search paths
set(CMAKE_FIND_ROOT_PATH "/usr/${TOOLCHAIN_PREFIX}" ${CMAKE_PREFIX_PATH})
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

# Remove prefix paths, as they were moved to the root path
unset(CMAKE_PREFIX_PATH)
