cmake_minimum_required(VERSION 3.10)
project(GasSimulation LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(SFML 2.5 COMPONENTS graphics window system REQUIRED)

# Указываем директории с заголовками
include_directories(include)

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

target_link_libraries(simulation PRIVATE sfml-graphics sfml-window sfml-system)

if(UNIX AND NOT APPLE)
    find_package(Threads REQUIRED)
    target_link_libraries(simulation PRIVATE Threads::Threads)
endif()
