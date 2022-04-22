cmake_minimum_required(VERSION 3.0.0)
project(Boids VERSION 0.1.0)

include(CTest)
enable_testing()

include_directories(/usr/include/SFML/)
link_directories(/usr/lib/aarch64-linux-gnu/)

add_executable(${PROJECT_NAME} main.cpp boid.cpp)
target_link_libraries (${PROJECT_NAME} sfml-graphics sfml-window sfml-system)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
