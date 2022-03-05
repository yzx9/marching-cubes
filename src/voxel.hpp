#pragma once
#include <vector>
#include <string>
#include <memory>
#include <limits>
#include <tiffio.h>

namespace voxel
{
    namespace _private
    {
        template <typename T>
        std::vector<std::vector<std::vector<T>>> read_tiff_imgs(std::string filePath);

        template <typename Tin, typename Tout>
        std::vector<std::vector<std::vector<Tout>>> normalize(std::vector<std::vector<std::vector<Tin>>> imgs);

        template <typename Tin, typename Tout, int Scale>
        std::vector<std::vector<std::vector<Tout>>> normalize(std::vector<std::vector<std::vector<Tin>>> imgs);
    }

    template <typename T>
    std::vector<std::vector<std::vector<T>>> read_from_tiff(std::string filePath)
    {
        using namespace _private;
        auto imgs = read_tiff_imgs<uint8>(filePath);
        auto voxels = normalize<uint8, T>(imgs);
        return std::move(voxels);
    }

    namespace _private
    {
        template <typename T>
        std::vector<std::vector<std::vector<T>>> read_tiff_imgs(std::string filePath)
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
        std::vector<std::vector<std::vector<Tout>>> normalize(std::vector<std::vector<std::vector<Tin>>> imgs)
        {
            std::vector<std::vector<std::vector<Tout>>> newImgs;
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

        template <typename Tin, typename Tout>
        std::vector<std::vector<std::vector<Tout>>> normalize(std::vector<std::vector<std::vector<Tin>>> imgs)
        {
            constexpr auto max = std::numeric_limits<Tin>::max();
            return std::move(normalize<Tin, Tout, max>(imgs));
        }
    }
}
