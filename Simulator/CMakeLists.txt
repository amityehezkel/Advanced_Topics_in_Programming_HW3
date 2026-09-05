if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    cmake_minimum_required(VERSION 3.15)
    set(VCPKG_MANIFEST_DIR "${CMAKE_CURRENT_LIST_DIR}/.." CACHE PATH
        "Assignment 3 vcpkg manifest directory")
    project(assignment_3_simulator LANGUAGES CXX)
    include(CTest)
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

find_package(TinyNPY CONFIG REQUIRED)
find_package(yaml-cpp CONFIG REQUIRED)
find_package(Threads REQUIRED)

add_library(simulator_core STATIC
    src/CommandLineParser.cpp
    src/ConfigurationLoader.cpp
    src/ErrorLogger.cpp
    src/FactoryRegistry.cpp
    src/Map3DImpl.cpp
    src/MapsComparison.cpp
    src/MockGPS.cpp
    src/MockLidar.cpp
    src/MockMovement.cpp
    src/OutputManager.cpp
    src/PluginLoader.cpp
    src/SharedLibrary.cpp
    src/SimulationManager.cpp
    src/SimulationReportWriter.cpp
    src/SimulationRunFactoryImpl.cpp
    src/SimulationRunImpl.cpp
    src/SimulatorApplication.cpp
)

target_include_directories(simulator_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/common_simulator/include
)

target_link_libraries(simulator_core
    PUBLIC
        common::common
        TinyNPY::TinyNPY
        yaml-cpp::yaml-cpp
        Threads::Threads
        ${CMAKE_DL_LIBS}
)

drone_warnings(simulator_core)

add_executable(simulator_212200943
    src/main.cpp
    src/MappingAlgorithmRegistration.cpp
    src/MissionControlRegistration.cpp
)

target_link_libraries(simulator_212200943 PRIVATE simulator_core)
set_target_properties(simulator_212200943 PROPERTIES ENABLE_EXPORTS ON)
drone_warnings(simulator_212200943)

if(BUILD_TESTING)
    find_package(GTest CONFIG REQUIRED)
    add_executable(simulator_tests tests/SimulatorTests.cpp)
    target_link_libraries(simulator_tests
        PRIVATE
            simulator_core
            GTest::gtest_main
    )
    drone_warnings(simulator_tests)
    add_test(NAME simulator_tests COMMAND simulator_tests)
endif()
