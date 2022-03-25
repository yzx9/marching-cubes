#pragma once
#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include "marchingCubes.hpp"
#include "Mesh.hpp"
#include "Vec.hpp"

namespace obj
{
    mesh::Mesh<float> read(const std::string &filePath)
    {
        std::ifstream stream;
        stream.open(filePath, std::ios::in);

        std::string s;
        std::vector<vec::Vec3<float>> vertices;
        std::vector<vec::Vec3<float>> normals;
        std::vector<vec::Vec3<vec::Vec3<int>>> faces;
        while (getline(stream, s))
        {
            if (s.starts_with('v'))
            {
                vec::Vec3<float> v;
                int i = 2;
                for (int k = 0; k < 3; k++)
                {
                    while (i < s.size() && !(s[i] == '-' || s[i] == '.' || (s[i] >= '0' && s[i] <= '9')))
                        i++;

                    int j = i;
                    while (i < s.size() && (s[i] == '-' || s[i] == '.' || (s[i] >= '0' && s[i] <= '9')))
                        i++;
                    v[k] = std::stof(s.substr(j, i - j));
                }

                if (s.starts_with("vn"))
                    normals.emplace_back(v);
                else
                    vertices.emplace_back(v);
            }

            if (s.starts_with('f'))
            {
                vec::Vec4<vec::Vec3<int>> f;
                int i = 2;
                for (int k = 0; k < 4; k++)
                {
                    while (i < s.size() && !(s[i] >= '0' && s[i] <= '9'))
                        i++;

                    int j = i;
                    while (i < s.size() && (s[i] >= '0' && s[i] <= '9'))
                        i++;
                    f[k][0] = std::stoi(s.substr(j, i - j));
                    i++;

                    if (s[i] != '/')
                    {
                        j = i;
                        while (i < s.size() && (s[i] >= '0' && s[i] <= '9'))
                            i++;
                        f[k][1] = std::stoi(s.substr(j, i - j));
                    }
                    else
                    {
                        f[k][1] = 0;
                        i++;
                    }

                    if (s[i] != ' ')
                    {
                        j = i;
                        while (i < s.size() && (s[i] >= '0' && s[i] <= '9'))
                            i++;
                        f[k][2] = std::stoi(s.substr(j, i - j));
                    }
                    else
                    {
                        f[k][2] = 0;
                        i++;
                    }
                }

                vec::Vec3<vec::Vec3<int>> f1{f[0], f[1], f[2]};
                vec::Vec3<vec::Vec3<int>> f2{f[1], f[2], f[3]};
                faces.emplace_back(f1);
                faces.emplace_back(f2);
            }
        }
        stream.close();

        mesh::Mesh<float> mesh;
        std::map<long, int> vertexMap;
        for (auto f : faces)
        {
            vec::Vec3<int> face;
            for (int i = 0; i < 3; i++)
            {
                auto v = f[i][0] - 1;
                auto n = f[i][2] - 1;
                long id = v << 32 + n;
                if (!vertexMap.count(id))
                {
                    vertexMap[id] = mesh.vertices.size();
                    mesh::Vertex<float> vertex{coord : vertices[v]};
                    if (n != -1)
                        vertex.normal = normals[n];
                    mesh.vertices.emplace_back(vertex);
                }
                face[i] = vertexMap[id];
            }
            mesh.faces.emplace_back(face);
        }
        return mesh;
    }

    template <typename T>
    void save(const std::string &filePath, const mesh::Mesh<T> &mesh)
    {
        std::ofstream stream;
        stream.open(filePath, std::ios::out);

        stream << "# List of vertices" << std::endl;
        for (auto &v : mesh.vertices)
            stream << "v "
                   << std::setprecision(4) << std::setw(7) << v.coord[0] << " "
                   << std::setprecision(4) << std::setw(7) << v.coord[1] << " "
                   << std::setprecision(4) << std::setw(7) << v.coord[2] << std::endl;
        stream << std::endl;

        stream << "# List of normals" << std::endl;
        for (auto &v : mesh.vertices)
            stream << "vn "
                   << std::setprecision(4) << std::setw(7) << v.normal[0] << " "
                   << std::setprecision(4) << std::setw(7) << v.normal[1] << " "
                   << std::setprecision(4) << std::setw(7) << v.normal[2] << std::endl;
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
