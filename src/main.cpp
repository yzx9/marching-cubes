#include <iostream>
#include <filesystem>
#include "tiff.hpp"
#include "marchingCubes.hpp"

int main()
{
    constexpr auto img = "../data/seg_ImgSoma_17302_00020-x_14992.3_y_21970.3_z_4344.8.tiff";
    auto filePath = std::filesystem::current_path().append(img);
    auto imgs = img_tiff::read(filePath);
    auto voxels = img_tiff::normalize<short, float, 255>(imgs);

    auto mesh = marching_cubes::extract<std::array<float, 3>>(voxels, 0.5);
    std::cout << mesh.size() << std::endl;

    return 0;
}
