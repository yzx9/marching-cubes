#include <iostream>
#include <filesystem>
#include "tiff.hpp"
#include "marchingCubes.hpp"

int main()
{
    constexpr auto img = "../data/seg_ImgSoma_17302_00020-x_14992.3_y_21970.3_z_4344.8.tiff";
    auto filePath = std::filesystem::current_path().append(img);
    img_tiff::read(filePath);
    return 0;
}
