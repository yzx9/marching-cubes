#pragma once
#include <array>
#include <functional>
#include <future>
#include <vector>
#include <memory>
#include <cmath>
#include "marchingCubesTables.hpp"
#include "voxel.hpp"

namespace marching_cubes
{
    template <typename T>
    using Vec3 = std::array<T, 3>;

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
    using Triangle = Vec3<Vertice<T>>;

    template <typename T>
    using Mesh = std::vector<Triangle<T>>;

    namespace _private
    {
        template <typename T>
        void calc_voxel(const voxel::Voxel<T> &vertices, float isovalue, const Vec3<int> &pos, Mesh<T> &out);

        template <typename T>
        Vertices<T> get_vertices(const voxel::Voxel<T> &vertices, const Vec3<int> &pos);

        template <typename T>
        std::array<Vertice<T>, 12> get_interpolation_points(const Vertices<T> &vertices, int edge, float isovalue);

        template <typename T>
        inline T interpolation(T isovalue, T x1, T x2);

        template <typename T>
        inline T interpolation(T isovalue, T f1, T f2, T x1, T x2);

        template <typename T>
        inline void normalize(Vec3<T> &vec);
    }

    template <typename T>
    Mesh<T> extract(const voxel::Voxel<T> &voxels, float isovalue)
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
            for (auto &tri : fut.get())
                mesh.emplace_back(tri);

        return std::move(mesh);
    }

    namespace _private
    {
        template <typename T>
        void calc_voxel(const voxel::Voxel<T> &voxels, float isovalue, const Vec3<int> &pos, Mesh<T> &out)
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
                out.emplace_back(Triangle<T>{
                    points[triangle[i + 0]],
                    points[triangle[i + 1]],
                    points[triangle[i + 2]],
                });
            }
        }

        template <typename T>
        Vertices<T> get_vertices(const voxel::Voxel<T> &voxels, const Vec3<int> &pos)
        {
            Vertices<T> v;
            for (auto i = 0; i < 8; i++)
            {
                const auto &[ox, oy, oz] = vertice_offsets[i];
                const auto x = std::get<0>(pos) + ox;
                const auto y = std::get<1>(pos) + oy;
                const auto z = std::get<2>(pos) + oz;
                const auto val = voxels[x][y][z];
                const auto coord = Vec3<T>{x, y, z};

                Vec3<T> normal;
                // TODO: following code like noodles
                normal[0] = x == 0 ? voxels[x + 1][y][z] - val
                            : x == voxels.size() - 1
                                ? val - voxels[x - 1][y][z]
                                : (voxels[x + 1][y][z] - voxels[x - 1][y][z]) / 2;

                normal[1] = y == 0 ? voxels[x][y + 1][z] - val
                            : y == voxels[x].size() - 1
                                ? val - voxels[x][y - 1][z]
                                : (voxels[x][y + 1][z] - voxels[x][y - 1][z]) / 2;

                normal[2] = z == 0 ? voxels[x][y][z + 1] - val
                            : z == voxels[x][y].size() - 1
                                ? val - voxels[x][y][z - 1]
                                : (voxels[x][y][z + 1] - voxels[x][y][z - 1]) / 2;

                normalize(normal);

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

                    Vec3<T> coord;
                    Vec3<T> normal;
                    for (auto j = 0; j < 3; j++)
                    {
                        coord[j] = interpolation<T>(isovalue, va.val, vb.val, va.coord[j], vb.coord[j]);
                        normal[j] = interpolation<T>(isovalue, va.normal[j], vb.normal[j]);
                    }

                    normalize(normal);
                    points[i] = Vertice<T>{
                        val : isovalue,
                        coord : coord,
                        normal : normal
                    };
                }
            }

            return std::move(points);
        };

        template <typename T>
        inline T interpolation(T isovalue, T x1, T x2)
        {
            return x1 + (x2 - x1) * isovalue;
        }

        template <typename T>
        inline T interpolation(T isovalue, T f1, T f2, T x1, T x2)
        {
            return x1 + (x2 - x1) * (isovalue - f1) / (f2 - f1);
        }

        template <typename T>
        inline void normalize(Vec3<T> &vec)
        {
            const auto norm = std::sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);
            for (int i = 0; i < 3; i++)
                vec[i] /= norm;
        }
    }
}
