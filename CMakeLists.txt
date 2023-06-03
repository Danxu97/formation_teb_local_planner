cmake_minimum_required(VERSION 3.16)
project(teb)

set(CMAKE_CXX_STANDARD 17)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(Boost REQUIRED COMPONENTS system thread graph)
find_package(SUITESPARSE REQUIRED)
find_package(G2O REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 3.3 REQUIRED)

include_directories(
        #/opt/ros/noetic/include/g2o/
        /usr/include/eigen3
        ${Eigen3_INCLUDE_DIR}
        ${SUITESPARSE_INCLUDE_DIRS}
        ${G2O_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
)

add_library(lib_teb
        src/teb_config.cpp
        src/obstacles.cpp
        src/visualization.cpp
        src/optimal_planner.cpp
        src/timed_elastic_band.cpp
)

add_executable(teb main.cpp)

target_link_libraries(
        teb
        lib_teb
        ${SUITESPARSE_LIBRARIES}
        ${G2O_LIBRARIES}
        -lboost_system -lboost_thread -lboost_graph
        ${OpenCV_LIBS}
        ${Eigen3_LIBS}
)
