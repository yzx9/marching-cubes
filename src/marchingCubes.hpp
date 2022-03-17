#pragma once
#include <array>
#include <cmath>
#include <functional>
#include <vector>
#include <memory>
#include "marchingCubesTables.hpp"
#include "Voxel.hpp"
#include "Vec.hpp"
#include "Mesh.hpp"

namespace marching_cubes
{
    using mesh::Mesh;
    using mesh::Vertex;
    using vec::Vec3;

    template <typename T>
    using Vertices = std::array<Vertex<T>, 8>;

    template <typename T>
    class MarchingCubes
    {
    public:
        MarchingCubes(const voxel::Voxels<T> &voxels, T isovalue);
        Mesh<T> &run();

    private:
        const T isovalue;
        const voxel::Voxels<T> &voxels;
        Mesh<T> mesh;
        std::vector<std::vector<std::vector<Vec3<int>>>> vertex_index;

        void calc_voxel(const Vec3<int> &pos);
        Vertices<T> get_vertices(const Vec3<int> &pos);
        std::array<int, 12> add_edge_vertices(const Vertices<T> &vertices, int edge);
    };

    template <typename T>
    Mesh<T> extract(const voxel::Voxels<T> &voxels, T isovalue)
    {
        MarchingCubes<T> alg(voxels, isovalue);
        return std::move(alg.run());
    }

    template <typename T>
    MarchingCubes<T>::MarchingCubes(const voxel::Voxels<T> &voxels, T isovalue) : voxels(voxels), isovalue(isovalue)
    {
        // initial vertices, set -1 as default
        for (auto x = 0; x < voxels.size() - 1; x++)
        {
            std::vector<std::vector<Vec3<int>>> vv;
            for (auto y = 0; y < voxels[0].size() - 1; y++)
            {
                std::vector<Vec3<int>> v;
                for (auto z = 0; z < voxels[0][0].size() - 1; z++)
                    v.emplace_back(Vec3<int>{-1, -1, -1});

                vv.emplace_back(v);
            }

            vertex_index.emplace_back(vv);
        }
    }

    template <typename T>
    Mesh<T> &MarchingCubes<T>::run()
    {
        // TODO[feat]: support async
        for (auto x = 0; x < voxels.size() - 1; x++)
            for (auto y = 0; y < voxels[0].size() - 1; y++)
                for (auto z = 0; z < voxels[0][0].size() - 1; z++)
                    calc_voxel({x, y, z});

        return mesh;
    }

    template <typename T>
    void MarchingCubes<T>::calc_voxel(const Vec3<int> &pos)
    {
        auto v = get_vertices(pos);
        auto index = 0;
        for (auto i = 0; i < 8; i++)
            index |= v[i].val < isovalue ? (0x01 << i) : 0x00;

        const auto edge = _private::edge_table[index];
        if (edge == 0)
            return;

        const auto &triangle = _private::triangle_table[index];
        const auto points = add_edge_vertices(v, edge);
        for (auto i = 0; triangle[i] != -1; i += 3)
            mesh.faces.emplace_back(Vec3<int>{
                points[triangle[i + 0]],
                points[triangle[i + 1]],
                points[triangle[i + 2]]});
    }

    template <typename T>
    Vertices<T> MarchingCubes<T>::get_vertices(const Vec3<int> &pos)
    {
        Vertices<T> v;
        for (auto i = 0; i < 8; i++)
        {
            const auto &[ox, oy, oz] = _private::vertex_offsets[i];
            const auto x = pos[0] + ox;
            const auto y = pos[1] + oy;
            const auto z = pos[2] + oz;
            v[i] = Vertex<T>{
                val : voxels[x][y][z],
                coord : Vec3<T>{static_cast<T>(x), static_cast<T>(y), static_cast<T>(z)},
                normal : voxel::get_normal<T>(voxels, x, y, z)
            };
        }

        return v;
    }

    template <typename T>
    std::array<int, 12> MarchingCubes<T>::add_edge_vertices(const Vertices<T> &vertices, int edge)
    {
        std::array<int, 12> points;
        for (auto i = 0; i < 12; i++)
        {
            if (((edge >> i) & 0x01) == 0x00)
                continue;

            const auto &[a, b, dir] = _private::edge_connection[i];
            const auto &va = vertices[a];
            const auto &vb = vertices[b];

            const auto min = vec::min<T>(va.coord, vb.coord);
            auto &index = vertex_index[min[0]][min[1]][min[2]][static_cast<int>(dir)];
            if (index == -1)
            {
                auto coord = vec::interpolate<T>(isovalue, va.val, vb.val, va.coord, vb.coord);
                auto normal = vec::interpolate<T>(isovalue, va.normal, vb.normal);

                // TODO[feat]: support async
                index = mesh.vertices.size();
                mesh.vertices.emplace_back(Vertex<T>{
                    val : isovalue,
                    coord : coord,
                    normal : vec::normalize(normal)
                });
            }

            points[i] = index;
        }

        return std::move(points);
    };
}
