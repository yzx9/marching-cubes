#pragma once
#include "Vec.hpp"

namespace mesh
{
    using vec::Vec3;

    template <typename T>
    struct Vertex
    {
        float val;
        Vec3<T> coord;
        Vec3<T> normal;
    };

    template <typename T>
    class Mesh
    {
    public:
        std::vector<Vertex<T>> vertices;
        std::vector<Vec3<int>> faces;
    };

    bool hasDegenerate(const Vec3<int> &face)
    {
        return (face[0] == face[1]) || (face[1] == face[2]) || (face[2] == face[0]);
    }

    template <typename T>
    Vertex<T> interpolate(double isovalue, const Vertex<T> &v1, const Vertex<T> &v2)
    {
        auto normal = vec::interpolate(isovalue, v1.normal, v2.normal);
        return Vertex<T>{
            val : v1.val + (v2.val - v1.val) * isovalue,
            coord : vec::interpolate(isovalue, v1.coord, v2.coord),
            normal : vec::normalize(normal),
        };
    }
}
