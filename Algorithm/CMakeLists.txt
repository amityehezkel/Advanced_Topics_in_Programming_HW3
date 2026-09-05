if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    cmake_minimum_required(VERSION 3.15)
    set(VCPKG_MANIFEST_DIR "${CMAKE_CURRENT_LIST_DIR}/.." CACHE PATH
        "Assignment 3 vcpkg manifest directory")
    project(assignment_3_algorithm LANGUAGES CXX)
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

add_library(Algorithm_212200943 SHARED
    src/MappingAlgorithmImpl.cpp
)

target_include_directories(Algorithm_212200943
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(Algorithm_212200943
    PRIVATE
        common::common
)

set_target_properties(Algorithm_212200943 PROPERTIES
    PREFIX ""
)

drone_warnings(Algorithm_212200943)
