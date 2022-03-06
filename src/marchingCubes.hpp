#pragma once
#include <array>
#include <functional>
#include <future>
#include <vector>
#include <memory>
#include <cmath>
#include "marchingCubesTables.hpp"
#include "voxel.hpp"
#include "Vec3.hpp"

namespace marching_cubes
{
    template <typename T>
    struct Vertice
    {
        float val;
        Vec3<T> coord;
        Vec3<T> normal;
    };

    template <typename T>
    using Vertices = std::array<Vertice<T>, 8>;

    template <typename T>
    class Mesh
    {
    public:
        std::vector<Vertice<T>> vertices;
        std::vector<Vec3<int>> faces;
    };

    namespace _private
    {
        template <typename T>
        void calc_voxel(const voxel::Voxels<T> &vertices, float isovalue, const Vec3<int> &pos, Mesh<T> &out);

        template <typename T>
        Vertices<T> get_vertices(const voxel::Voxels<T> &vertices, const Vec3<int> &pos);

        template <typename T>
        std::array<Vertice<T>, 12> get_interpolation_points(const Vertices<T> &vertices, int edge, float isovalue);
    }

    template <typename T>
    Mesh<T> extract(const voxel::Voxels<T> &voxels, float isovalue)
    {
        std::vector<std::future<Mesh<T>>> futures;
        for (auto x = 0; x < voxels.size() - 1; x++)
        {
            auto compute = [&](int x)
            {
                Mesh<T> mesh;
                for (auto y = 0; y < voxels[0].size() - 1; y++)
                    for (auto z = 0; z < voxels[0][0].size() - 1; z++)
                        _private::calc_voxel<T>(voxels, isovalue, {x, y, z}, mesh);

                return std::move(mesh);
            };

            futures.emplace_back(std::async(compute, x));
        }

        Mesh<T> mesh;
        for (auto &fut : futures)
        {
            auto submesh = fut.get();
            auto offset = mesh.vertices.size();
            for (auto &v : submesh.vertices)
                mesh.vertices.emplace_back(v);

            for (auto &f : submesh.faces)
                mesh.faces.emplace_back(Vec3<int>{offset + f[0], offset + f[1], offset + f[2]});
        }

        return std::move(mesh);
    }

    namespace _private
    {
        template <typename T>
        void calc_voxel(const voxel::Voxels<T> &voxels, float isovalue, const Vec3<int> &pos, Mesh<T> &out)
        {
            auto v = get_vertices<T>(voxels, pos);

            auto index = 0;
            for (auto i = 0; i < 8; i++)
                index |= v[i].val < isovalue ? (0x01 << i) : 0x00;

            const auto edge = edge_table[index];
            if (edge == 0)
                return;

            const auto &triangle = triangle_table[index];
            auto points = get_interpolation_points<T>(v, edge, isovalue);
            for (auto i = 0; triangle[i] != -1; i += 3)
            {
                auto s = out.vertices.size();
                out.vertices.emplace_back(points[triangle[i + 0]]);
                out.vertices.emplace_back(points[triangle[i + 1]]);
                out.vertices.emplace_back(points[triangle[i + 2]]);
                out.faces.emplace_back(Vec3<int>{s, s + 1, s + 2});
            }
        }

        template <typename T>
        Vertices<T> get_vertices(const voxel::Voxels<T> &voxels, const Vec3<int> &pos)
        {
            Vertices<T> v;
            for (auto i = 0; i < 8; i++)
            {
                const auto &[ox, oy, oz] = vertice_offsets[i];
                const auto x = std::get<0>(pos) + ox;
                const auto y = std::get<1>(pos) + oy;
                const auto z = std::get<2>(pos) + oz;
                const auto coord = Vec3<T>{static_cast<T>(x), static_cast<T>(y), static_cast<T>(z)};
                const auto val = voxels[x][y][z];
                const auto normal = voxel::get_normal<T>(voxels, x, y, z);

                v[i] = Vertice<T>{
                    val : val,
                    coord : coord,
                    normal : normal
                };
            }

            return v;
        }

        template <typename T>
        std::array<Vertice<T>, 12> get_interpolation_points(const Vertices<T> &vertices, int edge, float isovalue)
        {
            std::array<Vertice<T>, 12> points;
            for (auto i = 0; i < 12; i++)
            {
                if ((edge >> i) & 0x01)
                {
                    const auto &[a, b] = edge_connection[i];
                    const auto &va = vertices[a];
                    const auto &vb = vertices[b];

                    auto coord = vec3::interpolation<T>(isovalue, va.val, vb.val, va.coord, vb.coord);
                    auto normal = vec3::interpolation<T>(isovalue, va.normal, vb.normal);
                    vec3::normalize(normal);

                    points[i] = Vertice<T>{
                        val : isovalue,
                        coord : coord,
                        normal : normal
                    };
                }
            }

            return std::move(points);
        };
    }
}
