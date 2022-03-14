#include <chrono>
#include <iostream>
#include <filesystem>
#include <functional>
#include "marchingCubes.hpp"
#include "obj.hpp"
#include "quadricErrorMetrics.hpp"
#include "util.hpp"
#include "Voxel.hpp"

int main()
{
    constexpr auto img = "../data/seg_ImgSoma_17302_00020-x_14992.3_y_21970.3_z_4344.8.tiff";
    constexpr auto obj = "../tmp/seg_ImgSoma_17302_00020-x_14992.3_y_21970.3_z_4344.8.obj";

    auto imgFilePath = std::filesystem::current_path().append(img);
    auto voxelsRaw = voxel::read_from_tiff<float>(imgFilePath);
    auto voxels = voxel::smooth<float, 5>(voxelsRaw);

    auto mesh = util::run_with_duration(
        "Extract mesh", [](const auto &voxels)
        { return marching_cubes::extract<float>(voxels, 0.5); },
        voxels);

    util::run_with_duration(
        "Simplify mesh", [&mesh]()
        { return quadric_error_metrics::simplify(mesh, 0.5); });

    auto objFilePath = std::filesystem::current_path().append(obj);
    obj::save<float>(objFilePath, mesh);

    return 0;
}
