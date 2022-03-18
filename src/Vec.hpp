#pragma once
#include <array>
#include <cmath>
#include "Matrix.hpp"

namespace vec
{
    template <typename T>
    class Vec3
    {
    public:
        Vec3(){};
        Vec3(T x, T y, T z) : data({x, y, z}){};

        T &operator[](int i) { return data[i]; };
        const T &operator[](int i) const { return data[i]; };
        Vec3<T> operator+(const Vec3<T> &v) const { return Vec3<T>{data[0] + v[0], data[1] + v[1], data[2] + v[2]}; };
        Vec3<T> operator-(const Vec3<T> &v) const { return Vec3<T>{data[0] - v[0], data[1] - v[1], data[2] - v[2]}; };

        T &x() { return data[0]; };
        T &y() { return data[1]; };
        T &z() { return data[2]; };
        constexpr int size() const { return 3; };

    private:
        std::array<T, 3> data;
    };

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
    inline T norm2(const Vec3<T> &vec)
    {
        return vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2];
    }

    template <typename T>
    inline T norm(const Vec3<T> &vec) { return std::sqrt(norm2(vec)); }

    template <typename T>
    inline Vec3<T> normalize(const Vec3<T> &vec)
    {
        const auto n = norm(vec);
        return Vec3<T>{vec[0] / n, vec[1] / n, vec[2] / n};
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
    inline double distance(const Vec3<T> &v1, const Vec3<T> &v2) { return std::sqrt(distance2(v1, v2)); }

    template <typename T>
    inline Vec3<T> min(const Vec3<T> &v1, const Vec3<T> &v2)
    {
        return Vec3<T>{std::min(v1[0], v2[0]), std::min(v1[1], v2[1]), std::min(v1[2], v2[2])};
    }

    template <typename Tin, typename Tout>
    inline Vec3<Tout> cast(const Vec3<Tin> &v)
    {
        return Vec3<Tout>{static_cast<Tout>(v[0]), static_cast<Tout>(v[1]), static_cast<Tout>(v[2])};
    }

    template <typename T>
    class Vec4
    {
    public:
        Vec4(){};
        Vec4(T x, T y, T z, T w) : data({x, y, z, w}){};
        Vec4(const Vec3<T> &v, T w) : data({v[0], v[1], v[2], w}){};

        T &operator[](int i) { return data[i]; };
        const T &operator[](int i) const { return data[i]; };
        Vec4<T> operator+(const Vec4<T> &v) const { return Vec4<T>{data[0] + v[0], data[1] + v[1], data[2] + v[2], data[3] + v[3]}; };
        Vec4<T> operator-(const Vec4<T> &v) const { return Vec4<T>{data[0] - v[0], data[1] - v[1], data[2] - v[2], data[3] - v[3]}; };
        T operator*(const Vec4<T> &v) const { return data[0] * v[0] + data[1] * v[1] + data[2] * v[2]; };
        Vec4<T> operator*(const matrix::SymmetryMatrix4<T> &m) const
        {
            return Vec4<T>{
                data[0] * (m(0, 0) + m(0, 1) + m(0, 2) + m(0, 3)),
                data[1] * (m(1, 0) + m(1, 1) + m(1, 2) + m(1, 3)),
                data[2] * (m(2, 0) + m(2, 1) + m(2, 2) + m(2, 3)),
                data[3] * (m(3, 0) + m(3, 1) + m(3, 2) + m(3, 3))};
        };

        T &x() { return data[0]; };
        T &y() { return data[1]; };
        T &z() { return data[2]; };
        T &w() { return data[3]; };
        constexpr int size() const { return 4; };

    private:
        std::array<T, 4> data;
    };

    template <typename T>
    inline T norm2(const Vec4<T> &vec)
    {
        return vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2] + vec[3] * vec[3];
    }

    template <typename T>
    inline T norm(const Vec4<T> &vec) { return std::sqrt(norm2(vec)); }

    template <typename T>
    inline Vec4<T> normalize(const Vec4<T> &vec)
    {
        const auto n = norm(vec);
        return Vec4<T>{vec[0] / n, vec[1] / n, vec[2] / n, vec[3] / n};
    }

    template <typename T>
    inline double distance2(const Vec4<T> &v1, const Vec4<T> &v2)
    {
        return (v1[0] - v2[0]) * (v1[0] - v2[0]) +
               (v1[1] - v2[1]) * (v1[1] - v2[1]) +
               (v1[2] - v2[2]) * (v1[2] - v2[2]) +
               (v1[3] - v2[3]) * (v1[3] - v2[3]);
    }

    template <typename T>
    inline double distance(const Vec4<T> &v1, const Vec4<T> &v2) { return std::sqrt(distance2(v1, v2)); }
}
