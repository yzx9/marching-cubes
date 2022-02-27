#pragma once
#include <string>
#include <iostream>
#include <fstream>
#include "marchingCubes.hpp"

namespace obj
{
    template <typename Vec3>
    void save(const std::string &filePath, const marching_cubes::Mesh<Vec3> &mesh)
    {
        std::ofstream stream;
        stream.open(filePath, std::ios::out);

        stream << "# List of vertices" << std::endl;
        for (auto &tri : mesh)
            for (auto i = 0; i < 3; i++)
                stream << "v"
                       << " " << tri[i].coord[0]
                       << " " << tri[i].coord[1]
                       << " " << tri[i].coord[2] << std::endl;

        stream << "# List of normals" << std::endl;
        for (auto &tri : mesh)
            for (auto i = 0; i < 3; i++)
                stream << "vn"
                       << " " << tri[i].normal[0]
                       << " " << tri[i].normal[1]
                       << " " << tri[i].normal[2] << std::endl;

        stream << "# List of triangles" << std::endl;
        for (auto i = 0; i < mesh.size(); i++)
        {
            auto j = 3 * i;
            stream << "f"
                   << " " << j + 0 << "//" << j + 0
                   << " " << j + 1 << "//" << j + 1
                   << " " << j + 2 << "//" << j + 2 << std::endl;
        }

        stream.close();
    }
}
