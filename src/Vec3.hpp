#pragma once
#include <array>
#include <cmath>

namespace vec3
{
    template <typename T>
    using Vec3 = std::array<T, 3>;

    template <typename T>
    inline Vec3<T> product(const Vec3<T> &v1, const Vec3<T> &v2)
    {
        return Vec3<T>{
            v1[1] * v2[2] - v1[2] * v2[1],
            v1[2] * v2[0] - v1[0] * v2[2],
            v1[0] * v2[1] - v1[1] * v2[0],
        };
    }

    template <typename T>
    inline T norm(const Vec3<T> &vec)
    {
        return std::sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);
    }

    template <typename T>
    inline Vec3<T> normalize(const Vec3<T> &vec)
    {
        const auto norm = vec3::norm(vec);
        return Vec3<T>{vec[0] / norm, vec[1] / norm, vec[2] / norm};
    }

    template <typename T>
    inline Vec3<T> interpolate(double interpolation, const Vec3<T> &v1, const Vec3<T> &v2)
    {
        return Vec3<T>{
            v1[0] + (v2[0] - v1[0]) * interpolation,
            v1[1] + (v2[1] - v1[1]) * interpolation,
            v1[2] + (v2[2] - v1[2]) * interpolation,
        };
    }

    template <typename T>
    inline Vec3<T> interpolate(T isovalue, T f1, T f2, const Vec3<T> &v1, const Vec3<T> &v2)
    {
        const double interpolation = (isovalue - f1) / (f2 - f1);
        return interpolate(interpolation, v1, v2);
    }

    template <typename T>
    inline double distance2(const Vec3<T> &v1, const Vec3<T> &v2)
    {
        return (v1[0] - v2[0]) * (v1[0] - v2[0]) +
               (v1[1] - v2[1]) * (v1[1] - v2[1]) +
               (v1[2] - v2[2]) * (v1[2] - v2[2]);
    }

    template <typename T>
    inline double distance(const Vec3<T> &v1, const Vec3<T> &v2)
    {
        return std::sqrt(distance2(v1, v2));
    }

    template <typename T>
    inline Vec3<T> min(const Vec3<T> &v1, const Vec3<T> &v2)
    {
        return Vec3<T>{
            v1[0] < v2[0] ? v1[0] : v2[0],
            v1[1] < v2[1] ? v1[1] : v2[1],
            v1[2] < v2[2] ? v1[2] : v2[2]};
    }

    template <typename Tin, typename Tout>
    inline Vec3<Tout> cast(const Vec3<Tin> &v)
    {
        return Vec3<Tout>{
            static_cast<Tout>(v[0]),
            static_cast<Tout>(v[1]),
            static_cast<Tout>(v[2])};
    }
}
