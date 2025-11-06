# Cross-compile AxleStream for 64-bit ARM Linux (e.g. Jetson, Raspberry Pi 4/5, Graviton).
#   cmake -B build-arm -DCMAKE_TOOLCHAIN_FILE=cmake/aarch64-toolchain.cmake
# Requires the gcc-aarch64-linux-gnu / g++-aarch64-linux-gnu packages.
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

set(AXLE_CROSS_PREFIX "aarch64-linux-gnu-" CACHE STRING "Cross toolchain prefix")
set(CMAKE_C_COMPILER   ${AXLE_CROSS_PREFIX}gcc)
set(CMAKE_CXX_COMPILER ${AXLE_CROSS_PREFIX}g++)

if(DEFINED ENV{AXLE_SYSROOT})
  set(CMAKE_SYSROOT $ENV{AXLE_SYSROOT})
endif()

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# Cortex-A72/A76 class cores; override with -DCMAKE_CXX_FLAGS if targeting something else.
set(CMAKE_CXX_FLAGS_INIT "-march=armv8-a+crc")
