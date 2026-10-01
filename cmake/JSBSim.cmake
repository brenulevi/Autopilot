set(JSBSIM_SOURCE_DIR "" CACHE PATH "Optional existing JSBSim source tree (with aircraft, engine and systems)")

# Keep optional upstream tools out of this project's build.
set(BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(BUILD_PYTHON_MODULE OFF CACHE BOOL "" FORCE)
set(BUILD_JULIA_PACKAGE OFF CACHE BOOL "" FORCE)
set(BUILD_MATLAB_SFUNCTION OFF CACHE BOOL "" FORCE)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
set(CMAKE_DISABLE_FIND_PACKAGE_CxxTest TRUE)

if(JSBSIM_SOURCE_DIR)
    get_filename_component(jsbsim_SOURCE_DIR "${JSBSIM_SOURCE_DIR}" ABSOLUTE)
    if(NOT EXISTS "${jsbsim_SOURCE_DIR}/src/FGFDMExec.h")
        message(FATAL_ERROR "JSBSIM_SOURCE_DIR must point to a JSBSim source checkout")
    endif()
else()
    # The Git submodule pins JSBSim v1.3.1, including its aircraft data.
    set(jsbsim_SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}/../third_party/jsbsim")
    if(NOT EXISTS "${jsbsim_SOURCE_DIR}/src/FGFDMExec.h")
        message(FATAL_ERROR
            "JSBSim submodule is not initialized. Run from the project root:\n"
            "  git submodule update --init --recursive\n"
            "Or set JSBSIM_SOURCE_DIR to an existing JSBSim source checkout.")
    endif()
endif()

add_subdirectory("${jsbsim_SOURCE_DIR}" "${CMAKE_BINARY_DIR}/_deps/jsbsim-build" EXCLUDE_FROM_ALL)
set(AUTOPILOT_JSBSIM_DATA_DIR "${jsbsim_SOURCE_DIR}")
