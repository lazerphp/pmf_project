cmake_minimum_required(VERSION 3.16)
project(GasSimulation LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(SFML 3 REQUIRED COMPONENTS Graphics Window System)

# Собираем список исходных файлов
set(SOURCES
    main.cpp
    src/CameraController.cpp
    src/Particle.cpp
    src/Corridor.cpp
    src/Simulation.cpp
    src/Zone.cpp
    # Vector2.cpp не нужен, если реализация в заголовке, 
    # но если вынесли туда - добавьте сюда
)

add_executable(simulation ${SOURCES})

target_include_directories(simulation PRIVATE include)

target_link_libraries(simulation PRIVATE SFML::Graphics SFML::Window SFML::System)

if(UNIX AND NOT APPLE)
    find_package(Threads REQUIRED)
    target_link_libraries(simulation PRIVATE Threads::Threads)
endif()
