add_subdirectory(vendor/jsbsim)

add_executable(JSBSimAdapter
    src/Main.cpp
)

target_link_libraries(JSBSimAdapter
    PUBLIC
        libJSBSim
        Autopilot
)