#pragma once
#include "Vec3.hpp"

namespace mesh
{
    using vec3::Vec3;
    using Face = Vec3<int>;

    template <typename T>
    class Vertice
    {
    public:
        float val;
        Vec3<T> coord;
        Vec3<T> normal;
    };

    template <typename T>
    class Mesh
    {
    public:
        std::vector<Vertice<T>> vertices;
        std::vector<Face> faces;
    };

    bool hasDegenerate(const Face &face)
    {
        return (face[0] == face[1]) || (face[1] == face[2]) || (face[2] == face[0]);
    }

    template <typename T>
    Vertice<T> interpolation(double isovalue, const Vertice<T> &v1, const Vertice<T> v2)
    {
        auto normal = vec3::interpolation(isovalue, v1.normal, v2.normal);
        vec3::normalize(normal);
        return Vertice<T>{
            val : v1.val * isovalue + v2.val * (1 - isovalue),
            coord : vec3::interpolation(isovalue, v1.coord, v2.coord),
            normal : normal,
        };
    }
}
