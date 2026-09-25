include(FetchContent)

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
    add_subdirectory("${jsbsim_SOURCE_DIR}" "${CMAKE_BINARY_DIR}/_deps/jsbsim-build" EXCLUDE_FROM_ALL)
else()
    # JSBSim v1.3.1. Pin the source and verify the archive, rather than tracking master.
    FetchContent_Declare(jsbsim
        URL https://codeload.github.com/JSBSim-Team/jsbsim/zip/3b25f25e49b42d0489c04ac805674fc1450ca579
        URL_HASH SHA256=df57467a831cfa3ee3cadcb98d8291ce56bb11976e35dd9e93c94201ff1420d5
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    )
    FetchContent_MakeAvailable(jsbsim)
    # Only compile the upstream targets needed by our adapter.
    set_property(DIRECTORY "${jsbsim_SOURCE_DIR}" PROPERTY EXCLUDE_FROM_ALL TRUE)
endif()

set(AUTOPILOT_JSBSIM_DATA_DIR "${jsbsim_SOURCE_DIR}")
