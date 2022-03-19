#pragma once
#include <array>

namespace matrix
{
    template <typename T>
    class SymmetryMatrix4
    {
    private:
        std::array<T, 10> data;
        constexpr static std::array<int, 16> map{0, 1, 2, 3,
                                                 1, 4, 5, 6,
                                                 2, 5, 7, 8,
                                                 3, 6, 8, 9};

    public:
        SymmetryMatrix4(){};
        SymmetryMatrix4(T a, T b, T c, T d,
                        /**/ T e, T f, T g,
                        /*     */ T h, T i,
                        /*          */ T j) : data({a, b, c, d, e, f, g, h, i, j}){};

        SymmetryMatrix4<T> operator+(const SymmetryMatrix4<T> &m) const
        {
            SymmetryMatrix4<T> matrix;
            for (int i = 0; i < data.size(); i++)
                matrix.data[i] = data[i] + m.data[i];

            return matrix;
        };

        void operator+=(const SymmetryMatrix4<T> &m)
        {
            for (int i = 0; i < data.size(); i++)
                data[i] += m.data[i];
        };

        T &operator()(int i, int j) { return data[map[4 * i + j]]; };
        const T &operator()(int i, int j) const { return data[map[4 * i + j]]; };

        inline void fill(T val) { data.fill(val); };
    };
}
