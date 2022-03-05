#pragma once
#include <cmath>
#include <functional>
#include <vector>
#include <string>
#include <memory>
#include <limits>
#include <tiffio.h>

namespace voxel
{
    template <typename T>
    using Voxels = std::vector<std::vector<std::vector<T>>>;

    namespace _private
    {
        template <typename T>
        Voxels<T> read_tiff_imgs(std::string filePath);

        template <typename Tin, typename Tout, int Scale = std::numeric_limits<Tin>::max()>
        Voxels<Tout> normalize(Voxels<Tin> imgs);

        template <typename T, int Size>
        constexpr std::array<T, Size> generate_gaussian_vector(double sigma);
    }

    template <typename T>
    Voxels<T> read_from_tiff(std::string filePath)
    {
        using namespace _private;
        auto imgs = read_tiff_imgs<uint8>(filePath);
        return normalize<uint8, T>(imgs);
    }

    template <typename T, int Size>
    Voxels<T> smooth(const Voxels<T> &voxels)
    {
        const auto vec = _private::generate_gaussian_vector<T, Size>(0.8);

        // Sperate gaussian filter
        Voxels<T> src;
        Voxels<T> dst = voxels;
        for (int channel = 0; channel < 3; channel++)
        {
            src = dst;
            for (int i = 0; i < voxels.size() - Size; i++)
            {
                for (int j = 0; j < voxels[0].size() - Size; j++)
                {
                    for (int k = 0; k < voxels[0][0].size() - Size; k++)
                    {
                        T sum = 0;
                        for (int t = 0; t < Size; t++)
                        {
                            T val;
                            if (channel == 0)
                                val = src[i + t][j][k];
                            else if (channel == 1)
                                val = src[i][j + t][k];
                            else if (channel == 2)
                                val = src[i][j][k + t];

                            sum += vec[t] * val; // x
                        }

                        if (sum < 0)
                            sum = static_cast<T>(0);
                        else if (sum > 1)
                            sum = static_cast<T>(1);

                        dst[i][j][k] = sum;
                    }
                }
            }
        }

        return dst;
    }

    namespace _private
    {
        template <typename T>
        Voxels<T> read_tiff_imgs(std::string filePath)
        {
            using Row = std::vector<T>;
            using Img = std::vector<Row>;

            auto tif = TIFFOpen(filePath.c_str(), "r");
            auto page = TIFFNumberOfDirectories(tif);

            std::vector<Img> imgs;
            for (auto i = 0; i < page; i++)
            {
                TIFFSetDirectory(tif, i);

                int w;
                auto ret = TIFFGetField(tif, TIFFTAG_IMAGEWIDTH, &w); // TODO: handle ret

                int h;
                ret = TIFFGetField(tif, TIFFTAG_IMAGELENGTH, &h); // TODO: handle ret

                auto tmp = new uint32[w * h];
                TIFFReadRGBAImage(tif, w, h, tmp, 0);
                uint32 *pRow = tmp + (h - 1) * w;
                Img img;
                for (int i = 0; i < h; i++)
                {
                    Row row;
                    uint32 *pCol = pRow;
                    for (int j = 0; j < w; j++)
                    {
                        row.push_back(TIFFGetG(*pCol));
                        pCol++;
                    }
                    pRow -= w;
                    img.push_back(std::move(row));
                }

                imgs.push_back(std::move(img));
                delete[] tmp;
            }

            TIFFClose(tif);
            return std::move(imgs);
        }

        template <typename Tin, typename Tout, int Scale>
        Voxels<Tout> normalize(Voxels<Tin> imgs)
        {
            Voxels<Tout> newImgs;
            newImgs.reserve(imgs.size());
            for (auto &img : imgs)
            {
                std::vector<std::vector<Tout>> newImg;
                newImg.reserve(img.size());
                for (auto &row : img)
                {
                    std::vector<Tout> newRow;
                    newRow.reserve(row.size());
                    for (auto &pixel : row)
                    {
                        auto val = static_cast<Tout>(pixel) / Scale;
                        newRow.emplace_back(val);
                    }
                    newImg.emplace_back(std::move(newRow));
                }
                newImgs.emplace_back(std::move(newImg));
            }
            return std::move(newImgs);
        }

        template <typename T, int Size>
        constexpr std::array<T, Size> generate_gaussian_vector(double sigma)
        {
            std::array<T, Size> vec;
            double sum = 0;
            int origin = Size / 2;
            for (int i = 0; i < Size; i++)
            {
                // ignore coefficient
                T g = std::exp(-(i - origin) * (i - origin) / (2 * sigma * sigma));
                sum += g;
                vec[i] = g;
            }

            // normalize
            for (int i = 0; i < Size; i++)
                vec[i] /= sum;

            return vec;
        }
    }
}
