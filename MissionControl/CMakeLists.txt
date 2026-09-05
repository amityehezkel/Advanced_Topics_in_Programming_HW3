if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    cmake_minimum_required(VERSION 3.15)
    set(VCPKG_MANIFEST_DIR "${CMAKE_CURRENT_LIST_DIR}/.." CACHE PATH
        "Assignment 3 vcpkg manifest directory")
    project(assignment_3_mission_control LANGUAGES CXX)
    set(CMAKE_CXX_STANDARD 20)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_EXTENSIONS OFF)
    find_package(mp-units CONFIG REQUIRED)
    add_subdirectory(
        "${CMAKE_CURRENT_LIST_DIR}/../common"
        "${CMAKE_BINARY_DIR}/common"
    )
    function(drone_warnings target)
        target_compile_options(${target} PRIVATE -Wall -Wextra -Werror -pedantic)
    endfunction()
endif()

add_library(MissionControl_212200943 SHARED
    src/DroneControlImpl.cpp
    src/MissionControlImpl.cpp
    src/ScanResultToVoxels.cpp
    src/VerboseLogger.cpp
)

target_include_directories(MissionControl_212200943
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/common_mission_control/include
)

target_link_libraries(MissionControl_212200943
    PRIVATE
        common::common
)

set_target_properties(MissionControl_212200943 PROPERTIES
    PREFIX ""
)

drone_warnings(MissionControl_212200943)
