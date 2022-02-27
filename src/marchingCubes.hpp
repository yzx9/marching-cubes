#include <array>
#include <vector>
#include <memory>
#include "marchingCubesStateTables.hpp"

namespace marching_cubes
{
    namespace _private
    {
        float interpolation(const float &isovalue, float f1, float f2, float x1, float x2)
        {
            return x1 + (x2 - x1) * (isovalue - f1) / (f2 - f1);
        }

        template <typename Vec3>
        std::array<Vec3, 12> calc_points(const std::array<float, 8> &vertices, const int &edge, const float &isovalue,
                                         const float &x, const float &y, const float &z)
        {
            using namespace _private;

            std::array<Vec3, 12> points;
            const auto pos = std::array<float, 3>{x, y, z};
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
    }

    std::vector<std::array<std::array<float, 3>, 3>> extract(std::vector<std::vector<std::vector<float>>> vertices, float isovalue)
    {
        using namespace _private;

        using Vec3 = std::array<float, 3>;
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
                    std::array<float, 8> v;
                    for (auto i = 0; i < 8; i++)
                    {
                        auto &[a, b, c] = vertice_offsets[i];
                        v[i] = vertices[x + a][y + b][z + c];
                    }

                    auto index = 0;
                    for (auto i = 0; i < 8; i++)
                        index |= (v[i] < isovalue ? 0x01 : 0x00) << i;

                    const auto edge = edge_table[index];
                    if (edge == 0)
                        continue;

                    const auto &triangle = triangle_table[index];
                    auto points = calc_points<Vec3>(v, edge, isovalue, x, y, z);
                    for (auto i = 0; triangle[i] != -1; i += 3)
                    {
                        triangles.emplace_back(Triangle{
                            points[triangle[i + 0]],
                            points[triangle[i + 1]],
                            points[triangle[i + 2]]});
                    }
                }
            }
        }

        return std::move(triangles);
    }
}
