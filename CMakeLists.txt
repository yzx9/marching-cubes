cmake_minimum_required(VERSION 3.13)
project(marching_cubes)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_CXX_FLAGS "$ENV{CXXFLAGS} -O3 -Wall")

find_package(TIFF)

add_executable(marching_cubes
    src/main.cpp
    src/marchingCubes.hpp
    src/marchingCubesTables.hpp
    src/voxel.hpp
    src/obj.hpp
    src/util.hpp
)

target_link_libraries(marching_cubes TIFF::TIFF)
