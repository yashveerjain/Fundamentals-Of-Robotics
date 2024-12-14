cmake_minimum_required(VERSION 3.10)
project(home7)
set(CMAKE_CXX_STANDARD 17)

find_package(GTSAM REQUIRED)
find_package(Eigen3 3.4 REQUIRED)

include_directories(include/ ${GTSAM_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR})

add_library(data_handler src/data_handler.cpp)
add_library(custom_graph src/custom_graph_helper.cpp)

add_executable(main src/main.cpp)

target_link_libraries(main data_handler custom_graph gtsam)

