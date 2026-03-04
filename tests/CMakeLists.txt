include(FetchContent)

find_package(GTest QUIET)

if(NOT GTest_FOUND)
  FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
  )
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)
endif()

add_executable(pathlearn_tests
  core_types_test.cpp
  environment_collision_test.cpp
  dynamic_obstacle_test.cpp
  evaluation_test.cpp
  nn_sampler_test.cpp
  planner_test.cpp
  learning_test.cpp
  simulation_test.cpp
)
target_link_libraries(pathlearn_tests PRIVATE pathlearn_core GTest::gtest_main)
target_compile_definitions(pathlearn_tests PRIVATE
  PATHLEARN_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data"
)

include(GoogleTest)
gtest_discover_tests(pathlearn_tests)
