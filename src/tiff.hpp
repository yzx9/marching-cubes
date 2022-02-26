#include <vector>
#include <string>
#include <memory>
#include <iostream>
#include <tiffio.h>

namespace img_tiff
{
    std::vector<std::vector<std::vector<short>>> read(std::string filePath)
    {
        using Row = std::vector<short>;

        TIFF *tif = TIFFOpen(filePath.c_str(), "r");
        auto page = TIFFNumberOfDirectories(tif);

        std::vector<std::vector<Row>> imgs;
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
            std::vector<Row> img;
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
        return std::move(vec);
    }
}
