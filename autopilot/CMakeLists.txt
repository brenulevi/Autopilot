add_library(autopilot STATIC
    src/config.c
    src/autopilot.c
    src/mission.c
    src/path.c
    src/control/manual.c
    src/control/roll.c
    src/control/pitch.c
    src/control/airspeed.c
    src/guidance/bank.c
    src/guidance/pitch.c
    src/guidance/altitude.c
)
add_library(Autopilot::Core ALIAS autopilot)
target_include_directories(autopilot PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(autopilot PUBLIC Flight::Common)
if(NOT MSVC)
    target_link_libraries(autopilot PUBLIC m)
endif()
set_target_properties(autopilot PROPERTIES C_STANDARD 11 C_STANDARD_REQUIRED YES C_EXTENSIONS NO)
autopilot_warnings(autopilot)
