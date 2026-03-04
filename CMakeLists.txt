cmake_minimum_required(VERSION 3.16)

project(PathLearnNav LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(PATHLEARN_BUILD_TESTS "Build tests" ON)

add_library(pathlearn_core STATIC
  src/environment/simple_environment.cpp
  src/collision/simple_collision_checker.cpp
  src/dynamic/rule_based_dynamic_obstacle.cpp
  src/evaluation/simple_evaluator.cpp
  src/planner/simple_astar_planner.cpp
  src/planner/simple_rrt_planner.cpp
  src/learning/nn_sampler.cpp
  src/learning/cost_weight_learner.cpp
  src/simulation/simple_simulation.cpp
)
target_include_directories(pathlearn_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(pathlearn_app src/main.cpp)
target_link_libraries(pathlearn_app PRIVATE pathlearn_core)
target_compile_definitions(
  pathlearn_app PRIVATE PATHLEARN_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data"
)

add_executable(pathlearn_eval src/evaluation/eval_main.cpp)
target_link_libraries(pathlearn_eval PRIVATE pathlearn_core)
target_compile_definitions(
  pathlearn_eval PRIVATE PATHLEARN_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data"
)

option(PATHLEARN_WITH_SDL "Enable SDL2 visualization" ON)
option(PATHLEARN_WITH_SDL_TTF "Enable SDL2_ttf text rendering" ON)

if(PATHLEARN_WITH_SDL)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(SDL2 REQUIRED sdl2)
  set(PATHLEARN_SDL_TTF_FOUND FALSE)
  if(PATHLEARN_WITH_SDL_TTF)
    pkg_check_modules(SDL2_TTF QUIET SDL2_ttf)
    if(SDL2_TTF_FOUND)
      set(PATHLEARN_SDL_TTF_FOUND TRUE)
    else()
      message(WARNING "SDL2_ttf not found. Build without TTF text overlay.")
    endif()
  endif()
  add_library(pathlearn_vis STATIC
    src/visualization/sdl_visualizer.cpp
  )
  target_include_directories(pathlearn_vis
    PUBLIC
      ${CMAKE_CURRENT_SOURCE_DIR}/include
      ${SDL2_INCLUDE_DIRS}
  )
  target_link_libraries(pathlearn_vis PUBLIC ${SDL2_LIBRARIES})
  if(PATHLEARN_SDL_TTF_FOUND)
    target_include_directories(pathlearn_vis PUBLIC ${SDL2_TTF_INCLUDE_DIRS})
    target_link_libraries(pathlearn_vis PUBLIC ${SDL2_TTF_LIBRARIES})
    target_compile_definitions(pathlearn_vis PUBLIC PATHLEARN_WITH_SDL_TTF=1)
  else()
    target_compile_definitions(pathlearn_vis PUBLIC PATHLEARN_WITH_SDL_TTF=0)
  endif()
  target_compile_definitions(pathlearn_vis PUBLIC SDL_MAIN_HANDLED)
  target_compile_definitions(pathlearn_app PRIVATE PATHLEARN_WITH_SDL=1)
  target_compile_definitions(pathlearn_eval PRIVATE PATHLEARN_WITH_SDL=1)
  target_link_libraries(pathlearn_app PRIVATE pathlearn_vis)
  target_link_libraries(pathlearn_eval PRIVATE pathlearn_vis)
else()
  target_compile_definitions(pathlearn_app PRIVATE PATHLEARN_WITH_SDL=0)
  target_compile_definitions(pathlearn_eval PRIVATE PATHLEARN_WITH_SDL=0)
endif()

if(PATHLEARN_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
