cmake_minimum_required(VERSION 3.24)
project(Autopilot VERSION 0.1.0 LANGUAGES C)

option(AUTOPILOT_BUILD_SIM "Build the native JSBSim adapter and runner" ON)
include(CTest)

function(autopilot_warnings target)
    if(MSVC)
        target_compile_options(${target} PRIVATE /W4)
    else()
        target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
    endif()
endfunction()

add_subdirectory(autopilot)

if(AUTOPILOT_BUILD_SIM)
    enable_language(CXX)
    include(cmake/JSBSim.cmake)
    add_subdirectory(sim)
endif()

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()
