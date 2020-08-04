cmake_minimum_required(VERSION 3.17)
project(quadruped)

set(CMAKE_CXX_STANDARD 17)

find_package(towr 1.2 REQUIRED)
find_package(ifopt 2.0.1 REQUIRED)

add_executable(quadruped main.cpp model.cpp gallop.cpp trot.cpp turn.cpp idle.cpp trot_turn.cpp)
target_link_libraries(quadruped PUBLIC towr::towr ifopt::ifopt_ipopt)