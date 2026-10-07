add_library(autopilot STATIC src/autopilot.c src/control/roll.c src/control/pitch.c
    src/control/airspeed.c)
add_library(Autopilot::Core ALIAS autopilot)

target_include_directories(autopilot PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
set_target_properties(autopilot PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED YES
    C_EXTENSIONS NO)

if(MSVC)
    target_compile_options(autopilot PRIVATE /W4)
else()
    target_compile_options(autopilot PRIVATE -Wall -Wextra -Wpedantic)
endif()
