#include <chrono>
#include <iostream>
#include <filesystem>
#include <functional>
#include <vector>
#include "marchingCubes.hpp"
#include "obj.hpp"
#include "quadricErrorMetrics.hpp"
#include "util.hpp"
#include "Voxel.hpp"
#include "Mesh.hpp"

void extract_soma_mesh();
void simplify_test_mesh();
void simplify_human_mesh();

int main()
{
    extract_soma_mesh();
    return 0;
}

void extract_soma_mesh()
{
    constexpr auto img = "../data/seg_ImgSoma_17302_00020-x_14992.3_y_21970.3_z_4344.8.tiff";
    constexpr auto obj = "../tmp/seg_ImgSoma_17302_00020-x_14992.3_y_21970.3_z_4344.8.obj";

    auto imgFilePath = std::filesystem::current_path().append(img);
    auto voxelsRaw = util::run_with_duration(
        "Read voxels", [&imgFilePath]()
        { return voxel::read_from_tiff<float>(imgFilePath); });

    auto voxels = util::run_with_duration(
        "Smooth voxels", [](const auto &voxels)
        { return voxel::smooth<float, 5>(voxels); },
        voxelsRaw);

    auto mesh = util::run_with_duration(
        "Extract mesh", [](const auto &voxels)
        { return marching_cubes::extract<float>(voxels, 0.5); },
        voxels);

    util::run_with_duration(
        "Simplify mesh", [&mesh]()
        { return quadric_error_metrics::simplify(mesh, 0.3); });

    auto objFilePath = std::filesystem::current_path().append(obj);
    obj::save<float>(objFilePath, mesh);
}

void simplify_test_mesh()
{
    mesh::Mesh<float> mesh;

    std::vector<vec::Vec3<float>> vertices{
        vec::Vec3<float>{-2.0, -4.0, 0.0},
        vec::Vec3<float>{-2.0, +0.0, 0.0},
        vec::Vec3<float>{-2.0, +4.0, 0.0},
        vec::Vec3<float>{+0.0, -1.0, 1.0},
        vec::Vec3<float>{+0.0, +1.0, 1.0},
        vec::Vec3<float>{+2.0, -4.0, 0.0},
        vec::Vec3<float>{+2.0, +0.0, 0.0},
        vec::Vec3<float>{+2.0, +4.0, 0.0}};

    std::vector<vec::Vec3<int>> faces{
        // up
        vec::Vec3<int>{0, 3, 1},
        vec::Vec3<int>{1, 4, 2},
        vec::Vec3<int>{1, 3, 4},

        // down
        vec::Vec3<int>{3, 6, 4},
        vec::Vec3<int>{3, 5, 6},
        vec::Vec3<int>{4, 6, 7},

        // left
        vec::Vec3<int>{0, 5, 3},

        // right
        vec::Vec3<int>{2, 4, 7},

        // bottom
        vec::Vec3<int>{0, 1, 7},
        vec::Vec3<int>{1, 2, 7},
        vec::Vec3<int>{0, 6, 5},
        vec::Vec3<int>{0, 7, 6}};

    for (auto v : vertices)
        mesh.vertices.emplace_back(mesh::Vertex<float>{coord : v});

    for (auto f : faces)
        mesh.faces.emplace_back(f);

    util::run_with_duration(
        "Simplify mesh", [&mesh]()
        { return quadric_error_metrics::simplify(mesh, 0.3); });

    constexpr auto out = "../tmp/testCase.obj";
    auto objFilePath = std::filesystem::current_path().append(out);
    obj::save<float>(objFilePath, mesh);
}

void simplify_human_mesh()
{
    constexpr auto in = "../data/FinalBaseMesh.obj";
    constexpr auto out = "../tmp/FinalBaseMesh.obj";
    auto filePath = std::filesystem::current_path().append(in);
    auto mesh = obj::read(filePath);

    util::run_with_duration(
        "Simplify mesh", [&mesh]()
        { return quadric_error_metrics::simplify(mesh, 0.3); });

    auto objFilePath = std::filesystem::current_path().append(out);
    obj::save<float>(objFilePath, mesh);
}
