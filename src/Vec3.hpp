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
}
