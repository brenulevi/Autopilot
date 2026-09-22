add_library(Autopilot STATIC
    src/Control/PID.c
    src/Control/Control.c
)

target_include_directories(Autopilot
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)