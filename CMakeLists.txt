cmake_minimum_required(VERSION 3.16)
project(GasSimulation LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(SFML 3 REQUIRED COMPONENTS Graphics Window System)

set(CORE_SOURCES
    src/Particle.cpp
    src/PhysicsForces.cpp
    src/Corridor.cpp
    src/HeadlessSimulation.cpp
    src/JsonRunIO.cpp
    src/SimulationCore.cpp
    src/SimulationStats.cpp
    src/Zone.cpp
)

add_library(simulation_core STATIC ${CORE_SOURCES})
target_include_directories(simulation_core PUBLIC include)

add_library(simulation_gui STATIC
    src/CameraController.cpp
    src/Simulation.cpp
)
target_include_directories(simulation_gui PRIVATE include)
target_link_libraries(simulation_gui
    PUBLIC simulation_core
    PUBLIC SFML::Graphics SFML::Window SFML::System
)

add_executable(simulation main.cpp)
target_include_directories(simulation PRIVATE include)
target_link_libraries(simulation PRIVATE simulation_gui)
target_compile_definitions(simulation PRIVATE SIMULATION_WITH_GUI=1)

if(UNIX AND NOT APPLE)
    find_package(Threads REQUIRED)
    target_link_libraries(simulation_core PUBLIC Threads::Threads)
endif()
