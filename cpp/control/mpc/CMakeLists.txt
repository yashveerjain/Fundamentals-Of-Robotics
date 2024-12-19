cmake_minimum_required(VERSION 3.10)
project(mpc)

set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIR})

add_executable(mpc src/main.cpp)