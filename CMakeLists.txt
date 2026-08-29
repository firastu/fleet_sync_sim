cmake_minimum_required(VERSION 3.21)

project(fleet-sync-sim
        VERSION 0.1.0
        DESCRIPTION "Deterministic simulation of a distributed robot fleet with decentralized map synchronization"
        LANGUAGES CXX)

option(FLEET_BUILD_TESTS "Build the test tree" ON)
option(FLEET_WERROR "Treat compiler warnings as errors" ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Build type" FORCE)
endif()

# Warning flags applied to every first-party target (libraries, apps, tests).
function(fleet_target_warnings target)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(${target} PRIVATE
      -Wall
      -Wextra
      -Wpedantic
      -Wconversion
      -Wshadow
      -Wnon-virtual-dtor
      -Woverloaded-virtual)
    if(FLEET_WERROR)
      target_compile_options(${target} PRIVATE -Werror)
    endif()
  endif()
endfunction()

add_subdirectory(src)

if(FLEET_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
