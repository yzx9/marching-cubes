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
        Vertices<Vec3> get_vertices(const Voxel &vertices, const Pos &pos);

        template <typename Vec3>
        void calc_voxel(const Voxel &vertices, float isovalue, const Pos &pos, Mesh<Vec3> &out);

        template <typename Vec3>
        std::array<Vertice<Vec3>, 12> calc_points(const Vertices<Vec3> &vertices, int edge, float isovalue, const Pos &pos);

        inline float interpolation(float isovalue, float f1, float f2, float x1, float x2);
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
                index |= (v[i].val < isovalue ? 0x01 : 0x00) << i;

            const auto edge = edge_table[index];
            if (edge == 0)
                return;

            const auto &triangle = triangle_table[index];
            auto points = calc_points(v, edge, isovalue, pos);
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
                auto x = std::get<0>(pos) + ox;
                auto y = std::get<1>(pos) + oy;
                auto z = std::get<2>(pos) + oz;
                auto val = voxels[x][y][z];

                constexpr auto getNorm = [](int v, int mv, float val, const std::function<float(int)> &get)
                {
                    if (v == 0)
                        return get(v + 1) - val;

                    if (v == mv - 1)
                        return val - get(v - 1);

                    return (get(v + 1) - get(v - 1)) / 2;
                };

                Vec3 normal;
                if (x == 0)
                {
                    normal[0] = voxels[x + 1][y][z] - val;
                }
                else if (x == voxels.size() - 1)
                {
                    normal[0] = val - voxels[x - 1][y][z];
                }
                else
                {
                    normal[0] = (voxels[x + 1][y][z] - voxels[x - 1][y][z]) / 2;
                }

                if (y == 0)
                {
                    normal[1] = voxels[x][y + 1][z] - val;
                }
                else if (y == voxels[x].size() - 1)
                {
                    normal[1] = val - voxels[x][y - 1][z];
                }
                else
                {
                    normal[1] = (voxels[x][y + 1][z] - voxels[x][y - 1][z]) / 2;
                }

                if (z == 0)
                {
                    normal[2] = voxels[x][y][z + 1] - val;
                }
                else if (z == voxels[x][y].size() - 1)
                {
                    normal[2] = val - voxels[x][y][z - 1];
                }
                else
                {
                    normal[2] = (voxels[x][y][z + 1] - voxels[x][y][z - 1]) / 2;
                }

                auto norm = std::sqrt(normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]);
                for (int i = 0; i < 3; i++)
                    normal[i] /= norm;

                v[i] = Vertice<Vec3>{
                    val : val,
                    normal : normal
                };
            }

            return v;
        }

        template <typename Vec3>
        std::array<Vertice<Vec3>, 12> calc_points(const Vertices<Vec3> &vertices, int edge, float isovalue, const Pos &pos)
        {
            std::array<Vertice<Vec3>, 12> points;
            for (auto i = 0; i < 12; i++)
            {
                if ((edge >> i) & 0x01)
                {
                    auto &[a, b] = edge_connection[i];
                    const auto &va = vertices[a];
                    const auto &vb = vertices[b];

                    Vec3 coord;
                    Vec3 normal;
                    for (auto j = 0; j < 3; j++)
                    {
                        coord[j] = interpolation(isovalue, va.coord[j], vb.coord[j], pos[j], pos[j] + 1);
                        normal[j] = interpolation(isovalue, va.normal[j], vb.normal[j], pos[j], pos[j] + 1);
                    }

                    points[i] = Vertice<Vec3>{
                        coord : coord,
                        normal : normal
                    };
                }
            }

            return std::move(points);
        };

        inline float interpolation(float isovalue, float f1, float f2, float x1, float x2)
        {
            return x1 + (x2 - x1) * (isovalue - f1) / (f2 - f1);
        }
    }
}
