add_library(jsbsim_adapter STATIC src/jsbsim_adapter.cpp)
add_library(Autopilot::JSBSimAdapter ALIAS jsbsim_adapter)
target_include_directories(jsbsim_adapter PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_compile_features(jsbsim_adapter PUBLIC cxx_std_17)
set_target_properties(jsbsim_adapter PROPERTIES CXX_EXTENSIONS NO)
target_link_libraries(jsbsim_adapter PUBLIC Autopilot::Core PRIVATE libJSBSim)
autopilot_warnings(jsbsim_adapter)

configure_file(sim_config.h.in "${CMAKE_CURRENT_BINARY_DIR}/generated/sim_config.h" @ONLY)
add_executable(autopilot_sim src/main.cpp)
target_include_directories(autopilot_sim PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/generated")
target_link_libraries(autopilot_sim PRIVATE Autopilot::JSBSimAdapter)
set_target_properties(autopilot_sim PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin/$<CONFIG>"
    CXX_EXTENSIONS NO
)
autopilot_warnings(autopilot_sim)
