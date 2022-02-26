cmake_minimum_required(VERSION 3.13)
project(tiff_marching_cube)

set(CMAKE_CXX_STANDARD 11)

add_executable(tiff_marching_cube
    src/main.cpp
    src/tiff.hpp            src/tiff.cpp
    src/marchingCubes.hpp   src/marchingCubes.cpp
)
