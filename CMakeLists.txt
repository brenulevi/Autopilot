cmake_minimum_required(VERSION 3.22)
project(Autopilot VERSION 0.1.0 LANGUAGES C)

include(CTest)

add_subdirectory(autopilot)

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()
