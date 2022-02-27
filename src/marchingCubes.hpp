#include <array>
#include <vector>
#include <memory>
#include "marchingCubesTables.hpp"

namespace marching_cubes
{
    using Voxel = std::vector<std::vector<std::vector<float>>>;

    template <typename Point>
    using Triangle = std::array<Point, 3>;

    template <typename Point>
    using Mesh = std::vector<Triangle<Point>>;

    namespace _private
    {
        using Pos = std::array<int, 3>;

        template <typename Vec3>
        void calc_voxel(const Voxel &vertices, float isovalue, const Pos &pos, Mesh<Vec3> &out);

        template <typename Vec3>
        std::array<Vec3, 12> calc_points(const std::array<float, 8> &vertices, int edge, float isovalue, const Pos &pos);

        inline float interpolation(float isovalue, float f1, float f2, float x1, float x2);
    }

    template <typename Vec3>
    Mesh<Vec3> extract(const Voxel &vertices, float isovalue)
    {
        using namespace _private;
        using Triangle = std::array<Vec3, 3>;

        // TODO: perform precondition check, we assume vertices is an m*n*k
        auto m = vertices.size();
        auto n = vertices[0].size();
        auto k = vertices[0][0].size();

        std::vector<Triangle> triangles;
        for (auto x = 0; x < m - 1; x++)
        {
            for (auto y = 0; y < n - 1; y++)
            {
                for (auto z = 0; z < k - 1; z++)
                {
                    calc_voxel(vertices, isovalue, {x, y, z}, triangles);
                }
            }
        }

        return std::move(triangles);
    }

    namespace _private
    {
        template <typename Vec3>
        void calc_voxel(const Voxel &vertices, float isovalue, const Pos &pos, Mesh<Vec3> &out)
        {
            std::array<float, 8> v;
            for (auto i = 0; i < 8; i++)
            {
                const auto &[a, b, c] = vertice_offsets[i];
                v[i] = vertices[std::get<0>(pos) + a][std::get<1>(pos) + b][std::get<2>(pos) + c];
            }

            auto index = 0;
            for (auto i = 0; i < 8; i++)
                index |= (v[i] < isovalue ? 0x01 : 0x00) << i;

            const auto edge = edge_table[index];
            if (edge == 0)
                return;

            const auto &triangle = triangle_table[index];
            auto points = calc_points<Vec3>(v, edge, isovalue, pos);
            for (auto i = 0; triangle[i] != -1; i += 3)
            {
                out.emplace_back(Triangle<Vec3>{points[triangle[i + 0]],
                                                points[triangle[i + 1]],
                                                points[triangle[i + 2]]});
            }
        }

        template <typename Vec3>
        std::array<Vec3, 12> calc_points(const std::array<float, 8> &vertices, int edge, float isovalue, const Pos &pos)
        {
            std::array<Vec3, 12> points;
            for (auto i = 0; i < 12; i++)
            {
                auto &[a, b] = edge_connection[i];
                if ((edge >> i) & 0x01)
                {
                    Vec3 point;
                    for (auto j = 0; j < 3; j++)
                        point[j] = interpolation(isovalue, vertices[a], vertices[b], pos[j], pos[j] + 1);

                    points[i] = point;
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
