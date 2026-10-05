cmake_minimum_required(VERSION 3.16)
project(BallBalancingPlatform LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
  add_compile_options(/W4 /permissive- /utf-8 /wd4324)
else()
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Threads REQUIRED)

# Shared core: header-only, portable to the ESP32-S3 (kinematics, camera geometry).
add_library(bbp_core INTERFACE)
target_include_directories(bbp_core INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/core/include)

# Simulator: physics, actuators, camera rendering.
add_library(bbp_sim STATIC
  sim/src/ball.cpp
  sim/src/servo.cpp
  sim/src/platform_dynamics.cpp
  sim/src/renderer.cpp
  sim/src/world.cpp)
target_include_directories(bbp_sim PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/sim/include)
target_link_libraries(bbp_sim PUBLIC bbp_core Threads::Threads)

add_executable(bbp_sim_demo apps/sim_demo/main.cpp)
target_link_libraries(bbp_sim_demo PRIVATE bbp_sim)

enable_testing()
add_executable(bbp_tests tests/test_main.cpp tests/test_kinematics.cpp tests/test_ball.cpp
  tests/test_camera.cpp tests/test_system.cpp)
target_link_libraries(bbp_tests PRIVATE bbp_sim)
add_test(NAME bbp_tests COMMAND bbp_tests)

# Student framework: the public student library (framework/include/bbp/bbp.hpp) and the
# runtime that hosts track()/control(). bbp_runner adds the simulator runner's main().
add_library(bbp_framework STATIC
  framework/src/student_api.cpp
  framework/src/runtime.cpp
  framework/src/settings.cpp)
target_include_directories(bbp_framework PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/framework/include)
target_link_libraries(bbp_framework PUBLIC bbp_sim)

add_library(bbp_runner STATIC framework/src/runner_main.cpp)
target_link_libraries(bbp_runner PUBLIC bbp_framework)

# The project templates built through the same path the app uses (keeps them compiling).
add_executable(bbp_reference_runner app/bbp_app/templates/tracking.cpp app/bbp_app/templates/control.cpp)
target_link_libraries(bbp_reference_runner PRIVATE bbp_runner)
