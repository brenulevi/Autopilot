add_library(autopilot STATIC
    src/config/config.c
    src/autopilot/step.c
    src/mission/mission.c
    src/mission/path.c
    src/control/manual.c
    src/control/roll_attitude.c
    src/control/roll_rate.c
    src/control/pitch_attitude.c
    src/control/pitch_rate.c
    src/control/airspeed.c
    src/control/yaw_rate.c
    src/control/turn_coordination.c
    src/guidance/bank_limit.c
    src/guidance/pitch_limit.c
    src/guidance/altitude.c
)
add_library(Autopilot::Core ALIAS autopilot)
target_include_directories(autopilot PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_include_directories(autopilot PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
target_link_libraries(autopilot PUBLIC Flight::Common)
if(NOT MSVC)
    target_link_libraries(autopilot PUBLIC m)
endif()
set_target_properties(autopilot PROPERTIES C_STANDARD 11 C_STANDARD_REQUIRED YES C_EXTENSIONS NO)
autopilot_warnings(autopilot)
