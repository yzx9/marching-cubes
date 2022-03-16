#pragma once
#include "Vec3.hpp"

namespace mesh
{
    using vec3::Vec3;
    using Face = Vec3<int>;

    template <typename T>
    class Vertex
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
        std::vector<Vertex<T>> vertices;
        std::vector<Face> faces;
    };

    bool hasDegenerate(const Face &face)
    {
        return (face[0] == face[1]) || (face[1] == face[2]) || (face[2] == face[0]);
    }

    template <typename T>
    Vertex<T> interpolate(double isovalue, const Vertex<T> &v1, const Vertex<T> v2)
    {
        auto normal = vec3::interpolate(isovalue, v1.normal, v2.normal);
        return Vertex<T>{
            val : v1.val * isovalue + v2.val * (1 - isovalue),
            coord : vec3::interpolate(isovalue, v1.coord, v2.coord),
            normal : vec3::normalize(normal),
        };
    }
}
