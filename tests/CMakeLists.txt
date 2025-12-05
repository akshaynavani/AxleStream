include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.tar.gz
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

include(GoogleTest)

set(AXLE_TEST_SOURCES
  test_spsc_queue.cpp
  test_latency.cpp
  test_log_format.cpp
  test_sim_sources.cpp
  test_processing.cpp
  test_pipeline.cpp
)

add_executable(axle_tests ${AXLE_TEST_SOURCES})
target_link_libraries(axle_tests PRIVATE axle_core GTest::gtest_main)
gtest_discover_tests(axle_tests DISCOVERY_TIMEOUT 30)
