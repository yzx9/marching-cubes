#pragma once
#include <array>

namespace vec3
{
    template <typename T>
    using Vec3 = std::array<T, 3>;

    template <typename T>
    inline void normalize(std::array<T, 3> &vec)
    {
        const auto norm = std::sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);
        for (int i = 0; i < 3; i++)
            vec[i] /= norm;
    }

    template <typename T>
    inline Vec3<T> interpolation(T isovalue, const Vec3<T> &v1, const Vec3<T> &v2)
    {
        Vec3<T> vec;
        for (auto i = 0; i < 3; i++)
            vec[i] = v1[i] + (v2[i] - v1[i]) * isovalue;

        return vec;
    }

    template <typename T>
    inline Vec3<T> interpolation(T isovalue, T f1, T f2, const Vec3<T> &v1, const Vec3<T> &v2)
    {
        Vec3<T> vec;
        const auto inter = (isovalue - f1) / (f2 - f1);
        for (auto j = 0; j < 3; j++)
            vec[j] = v1[j] + (v2[j] - v1[j]) * inter;

        return vec;
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
