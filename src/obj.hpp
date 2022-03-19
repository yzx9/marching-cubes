#pragma once
#include <string>
#include <iostream>
#include <fstream>
#include "marchingCubes.hpp"

namespace obj
{
    template <typename T>
    void save(const std::string &filePath, const marching_cubes::Mesh<T> &mesh)
    {
        std::ofstream stream;
        stream.open(filePath, std::ios::out);

        stream << "# List of vertices" << std::endl;
        for (auto &v : mesh.vertices)
            stream << "v " << v.coord[0] << " " << v.coord[1] << " " << v.coord[2] << std::endl;
        stream << std::endl;

        stream << "# List of normals" << std::endl;
        for (auto &v : mesh.vertices)
            stream << "vn " << v.normal[0] << " " << v.normal[1] << " " << v.normal[2] << std::endl;
        stream << std::endl;

        stream << "# List of faces" << std::endl;
        for (auto &f : mesh.faces)
            stream << "f"
                   << " " << f[0] + 1 << "//" << f[0] + 1
                   << " " << f[1] + 1 << "//" << f[1] + 1
                   << " " << f[2] + 1 << "//" << f[2] + 1
                   << std::endl;

        stream.close();
    }
}
