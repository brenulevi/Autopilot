cmake_minimum_required(VERSION 3.22)
project(Autopilot VERSION 0.1.0 LANGUAGES C)

option(AUTOPILOT_BUILD_SIM "Build the JSBSim controller simulator" OFF)
include(CTest)

add_subdirectory(autopilot)

if(AUTOPILOT_BUILD_SIM)
    enable_language(CXX)
    add_subdirectory(sim)
endif()

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()
