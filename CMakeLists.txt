cmake_minimum_required(VERSION 3.13)
project(tiff_marching_cube)

set(CMAKE_CXX_STANDARD 17)

find_package(TIFF)

add_executable(tiff_marching_cube
    src/main.cpp
    src/tiff.hpp
    src/marchingCubes.hpp
    src/marchingCubesTables.hpp
)

target_link_libraries(tiff_marching_cube TIFF::TIFF)
