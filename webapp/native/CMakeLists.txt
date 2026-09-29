cmake_minimum_required(VERSION 3.22)
project(MissionPreview LANGUAGES C)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
add_executable(mission_preview preview.c
    ../../autopilot/src/mission/mission.c
    ../../autopilot/src/mission/path.c
    ../../common/src/crc32.c)
target_include_directories(mission_preview PRIVATE ../../autopilot/include ../../common/include)
if(NOT MSVC)
  target_compile_options(mission_preview PRIVATE -Wall -Wextra -Wpedantic)
  target_link_libraries(mission_preview PRIVATE m)
endif()
set_target_properties(mission_preview PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
foreach(config DEBUG RELEASE RELWITHDEBINFO MINSIZEREL)
  set_target_properties(mission_preview PROPERTIES RUNTIME_OUTPUT_DIRECTORY_${config} "${CMAKE_BINARY_DIR}/bin")
endforeach()
