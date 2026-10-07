add_executable(roll_controller_test roll_controller_test.c)
target_link_libraries(roll_controller_test PRIVATE Autopilot::Core)
set_target_properties(roll_controller_test PROPERTIES
    C_STANDARD 11 C_STANDARD_REQUIRED YES C_EXTENSIONS NO)
add_test(NAME autopilot.roll_controller COMMAND roll_controller_test)

add_executable(pitch_controller_test pitch_controller_test.c)
target_link_libraries(pitch_controller_test PRIVATE Autopilot::Core)
set_target_properties(pitch_controller_test PROPERTIES
    C_STANDARD 11 C_STANDARD_REQUIRED YES C_EXTENSIONS NO)
add_test(NAME autopilot.pitch_controller COMMAND pitch_controller_test)

add_executable(airspeed_controller_test airspeed_controller_test.c)
target_link_libraries(airspeed_controller_test PRIVATE Autopilot::Core)
set_target_properties(airspeed_controller_test PROPERTIES
    C_STANDARD 11 C_STANDARD_REQUIRED YES C_EXTENSIONS NO)
add_test(NAME autopilot.airspeed_controller COMMAND airspeed_controller_test)

add_executable(autopilot_api_test autopilot_api_test.c)
target_link_libraries(autopilot_api_test PRIVATE Autopilot::Core)
set_target_properties(autopilot_api_test PROPERTIES
    C_STANDARD 11 C_STANDARD_REQUIRED YES C_EXTENSIONS NO)
add_test(NAME autopilot.public_api COMMAND autopilot_api_test)

if(AUTOPILOT_BUILD_SIM)
    find_package(Python3 QUIET COMPONENTS Interpreter)
    if(Python3_Interpreter_FOUND)
        add_test(NAME sim.c172x_roll_hold
            COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test_roll_sim.py"
                "$<TARGET_FILE:attitude_hold_sim>")
        add_test(NAME sim.c172x_pitch_hold
            COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test_pitch_sim.py"
                "$<TARGET_FILE:attitude_hold_sim>")
        add_test(NAME sim.c172x_airspeed_hold
            COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test_airspeed_sim.py"
                "$<TARGET_FILE:attitude_hold_sim>")
    endif()
endif()
