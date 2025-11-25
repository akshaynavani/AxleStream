cmake_minimum_required(VERSION 3.16)
project(AxleStream VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(AXLE_BUILD_TESTS "Build unit tests" ON)
option(AXLE_WITH_SOCKETCAN "Enable SocketCAN input (Linux only)" OFF)
option(AXLE_NATIVE_ARCH "Tune for the host CPU (-mcpu/-march=native)" OFF)

find_package(Threads REQUIRED)

add_library(axle_warnings INTERFACE)
target_compile_options(axle_warnings INTERFACE
  $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wno-sign-conversion>)

if(AXLE_NATIVE_ARCH)
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    add_compile_options(-mcpu=native)
  else()
    add_compile_options(-march=native)
  endif()
endif()

set(AXLE_CORE_SOURCES
  src/core/latency.cpp
  src/core/thread_util.cpp
  src/sources/vehicle_model.cpp
  src/sources/sim_lidar.cpp
  src/sources/sim_imu_can.cpp
  src/sources/socketcan_source.cpp
  src/io/log_codec.cpp
  src/io/log_writer.cpp
  src/io/log_reader.cpp
  src/io/jsonl_exporter.cpp
  src/processing/lidar_processor.cpp
  src/processing/imu_filter.cpp
  src/processing/can_decoder.cpp
  src/processing/fusion.cpp
  src/pipeline/pipeline.cpp
)

add_library(axle_core ${AXLE_CORE_SOURCES})
target_include_directories(axle_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(axle_core PUBLIC Threads::Threads PRIVATE axle_warnings)

if(AXLE_WITH_SOCKETCAN)
  if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
    message(WARNING "AXLE_WITH_SOCKETCAN requested on ${CMAKE_SYSTEM_NAME}; SocketCAN is Linux-only, disabling")
  else()
    target_compile_definitions(axle_core PUBLIC AXLE_WITH_SOCKETCAN=1)
  endif()
endif()

if(AXLE_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
