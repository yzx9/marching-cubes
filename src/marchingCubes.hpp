#pragma once
#include <array>
#include <functional>
#include <future>
#include <vector>
#include <memory>
#include <cmath>
#include "marchingCubesTables.hpp"

namespace marching_cubes
{
    using Pos = std::array<int, 3>;
    using Voxel = std::vector<std::vector<std::vector<float>>>;

    template <typename Vec3>
    struct Vertice
    {
        float val;
        Vec3 coord;
        Vec3 normal;
    };

    template <typename Vec3>
    using Vertices = std::array<Vertice<Vec3>, 8>;

    template <typename Vec3>
    using Triangle = std::array<Vertice<Vec3>, 3>;

    template <typename Vec3>
    using Mesh = std::vector<Triangle<Vec3>>;

    namespace _private
    {
        template <typename Vec3>
        void calc_voxel(const Voxel &vertices, float isovalue, const Pos &pos, Mesh<Vec3> &out);

        template <typename Vec3>
        Vertices<Vec3> get_vertices(const Voxel &vertices, const Pos &pos);

        template <typename Vec3>
        std::array<Vertice<Vec3>, 12> get_interpolation_points(const Vertices<Vec3> &vertices, int edge, float isovalue);

        inline float interpolation(float isovalue, float x1, float x2);
        inline float interpolation(float isovalue, float f1, float f2, float x1, float x2);

        template <typename Vec3>
        inline void normalize(Vec3 &vec);
    }

    template <typename Vec3>
    Mesh<Vec3> extract(const Voxel &voxels, float isovalue)
    {
        using namespace _private;

        const auto max = std::array<int, 3>{voxels.size(), voxels[0].size(), voxels[0][0].size()};
        std::vector<std::future<Mesh<Vec3>>> futures;
        futures.reserve(max[0] - 1);
        for (auto x = 0; x < max[0] - 1; x++)
        {
            auto compute = [&](int x)
            {
                Mesh<Vec3> mesh;
                for (auto y = 0; y < max[1] - 1; y++)
                    for (auto z = 0; z < max[2] - 1; z++)
                        calc_voxel(voxels, isovalue, {x, y, z}, mesh);

                return std::move(mesh);
            };

            futures.emplace_back(std::async(compute, x));
        }

        for (auto &fut : futures)
            fut.wait();

        Mesh<Vec3> mesh;
        for (auto &fut : futures)
        {
            auto submesh = fut.get();
            for (auto &tri : submesh)
                mesh.emplace_back(tri);
        }

        return std::move(mesh);
    }

    namespace _private
    {
        template <typename Vec3>
        void calc_voxel(const Voxel &voxels, float isovalue, const Pos &pos, Mesh<Vec3> &out)
        {
            auto v = get_vertices<Vec3>(voxels, pos);

            auto index = 0;
            for (auto i = 0; i < 8; i++)
                index |= v[i].val < isovalue ? (0x01 << i) : 0x00;

            const auto edge = edge_table[index];
            if (edge == 0)
                return;

            const auto &triangle = triangle_table[index];
            auto points = get_interpolation_points(v, edge, isovalue);
            for (auto i = 0; triangle[i] != -1; i += 3)
            {
                out.emplace_back(Triangle<Vec3>{
                    points[triangle[i + 0]],
                    points[triangle[i + 1]],
                    points[triangle[i + 2]],
                });
            }
        }

        template <typename Vec3>
        Vertices<Vec3> get_vertices(const Voxel &voxels, const Pos &pos)
        {
            Vertices<Vec3> v;
            for (auto i = 0; i < 8; i++)
            {
                const auto &[ox, oy, oz] = vertice_offsets[i];
                const auto x = std::get<0>(pos) + ox;
                const auto y = std::get<1>(pos) + oy;
                const auto z = std::get<2>(pos) + oz;
                const auto val = voxels[x][y][z];
                const auto coord = Vec3{x, y, z};

                Vec3 normal;
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

                v[i] = Vertice<Vec3>{
                    val : val,
                    coord : coord,
                    normal : normal
                };
            }

            return v;
        }

        template <typename Vec3>
        std::array<Vertice<Vec3>, 12> get_interpolation_points(const Vertices<Vec3> &vertices, int edge, float isovalue)
        {
            std::array<Vertice<Vec3>, 12> points;
            for (auto i = 0; i < 12; i++)
            {
                if ((edge >> i) & 0x01)
                {
                    const auto &[a, b] = edge_connection[i];
                    const auto &va = vertices[a];
                    const auto &vb = vertices[b];

                    Vec3 coord;
                    Vec3 normal;
                    for (auto j = 0; j < 3; j++)
                    {
                        coord[j] = interpolation(isovalue, va.val, vb.val, va.coord[j], vb.coord[j]);
                        normal[j] = interpolation(isovalue, va.normal[j], vb.normal[j]);
                    }

                    normalize(normal);
                    points[i] = Vertice<Vec3>{
                        val : isovalue,
                        coord : coord,
                        normal : normal
                    };
                }
            }

            return std::move(points);
        };

        inline float interpolation(float isovalue, float x1, float x2)
        {
            return x1 + (x2 - x1) * isovalue;
        }

        inline float interpolation(float isovalue, float f1, float f2, float x1, float x2)
        {
            return x1 + (x2 - x1) * (isovalue - f1) / (f2 - f1);
        }

        template <typename Vec3>
        inline void normalize(Vec3 &vec)
        {
            const auto norm = std::sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);
            for (int i = 0; i < 3; i++)
                vec[i] /= norm;
        }
    }
}
