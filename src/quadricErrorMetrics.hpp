#pragma once
#include <cmath>
#include <queue>
#include <tuple>
#include <vector>
#include "Mesh.hpp"
#include "unordered_map"

namespace quadric_error_metrics
{
    using mesh::Mesh;
    constexpr int INVALID = -1;

    struct Pair
    {
        int v1;
        int v2;
        int version; // invalid when any vertex change
        double quadricError;
        bool operator<(const Pair &c) const { return quadricError > c.quadricError; }
    };

    template <typename T>
    class QuadricErrorMetrics
    {
    public:
        QuadricErrorMetrics(Mesh<T> &mesh);
        void simplify(int N);

    private:
        Mesh<T> &mesh;
        std::vector<std::vector<int>> vertexFaces;
        std::vector<int> vertexVersions;
        std::priority_queue<Pair> pairs;

        void build_pairs();
        void contract_pair(const Pair &pair);
        void update_quadric_error(Pair &pair);
        mesh::Vertex<T> get_best_vertex(const Pair &pair);
        void tidy_mesh();
    };

    template <typename T>
    void simplify(Mesh<T> &mesh, double simplifyPercent)
    {
        const auto simplifyN = std::ceil(mesh.vertices.size() * simplifyPercent);
        QuadricErrorMetrics<T> qem(mesh);
        qem.simplify(simplifyN);
    };

    template <typename T>
    QuadricErrorMetrics<T>::QuadricErrorMetrics(Mesh<T> &mesh) : mesh(mesh)
    {
        vertexFaces.reserve(mesh.vertices.size());
        vertexVersions.reserve(mesh.vertices.size());
        for (int i = 0; i < mesh.vertices.size(); i++)
        {
            vertexFaces.emplace_back(std::vector<int>{});
            vertexVersions.emplace_back(0);
        }

        build_pairs();
    }

    template <typename T>
    void QuadricErrorMetrics<T>::simplify(int simplifyN)
    {
        while (simplifyN && !pairs.empty())
        {
            auto p = pairs.top();
            pairs.pop();
            if (vertexVersions[p.v1] == INVALID ||
                vertexVersions[p.v2] == INVALID ||
                vertexVersions[p.v1] + vertexVersions[p.v2] != p.version)
                continue;

            contract_pair(p);
            simplifyN--;
        }

        tidy_mesh();
    };

    template <typename T>
    void QuadricErrorMetrics<T>::build_pairs()
    {
        const int n = mesh.faces.size() * 3;
        std::unordered_map<long, Pair> pairMap;
        for (auto i = 0; i < mesh.faces.size(); i++)
        {
            const auto &face = mesh.faces[i];
            if (mesh::hasDegenerate(face))
                continue;

            std::array<std::tuple<int, int>, 3> faceEdges{
                std::tuple<int, int>{face[0], face[1]}, {face[1], face[2]}, {face[2], face[0]}};
            for (auto j = 0; j < faceEdges.size(); j++)
            {
                auto [v1, v2] = faceEdges[j];
                if (v1 > v2)
                    std::swap(v1, v2);

                long id = v1 * n + v2;
                if (pairMap.count(id) == 0)
                    pairMap[id] = {v1 : v1, v2 : v2, version : vertexVersions[v1] + vertexVersions[v2]};
            }

            for (auto v : face)
                vertexFaces[v].emplace_back(i);
        }

        for (auto &[_, pair] : pairMap)
        {
            update_quadric_error(pair);
            pairs.push(pair);
        }
    }

    template <typename T>
    void QuadricErrorMetrics<T>::contract_pair(const Pair &pair)
    {
        mesh.vertices[pair.v1] = get_best_vertex(pair);
        vertexVersions[pair.v1]++;
        vertexVersions[pair.v2] = INVALID;

        // merge faces from v2 to v1
        for (auto faceId : vertexFaces[pair.v2])
        {
            auto &face = mesh.faces.at(faceId);
            auto flag = true;
            for (auto &v : face)
            {
                if (v == pair.v1)
                    flag = false;

                if (v == pair.v2)
                    v = pair.v1;
            }

            if (flag)
                vertexFaces[pair.v1].emplace_back(faceId);
        }

        //  insert new pairs
        for (auto faceId : vertexFaces[pair.v1])
        {
            const auto &face = mesh.faces.at(faceId);
            if (mesh::hasDegenerate(face))
                continue;

            std::array<std::tuple<int, int>, 3> faceEdges{
                std::tuple<int, int>{face[0], face[1]}, {face[1], face[2]}, {face[2], face[0]}};
            for (auto [v1, v2] : faceEdges)
            {
                if (v1 > v2)
                    std::swap(v1, v2);

                Pair pair{
                    v1 : v1,
                    v2 : v2,
                    version : vertexVersions[v1] + vertexVersions[v2],
                };
                update_quadric_error(pair);
                pairs.emplace(pair);
            }
        }
    };

    template <typename T>
    void QuadricErrorMetrics<T>::update_quadric_error(Pair &pair)
    {
        pair.quadricError = std::rand(); // TODO
    };

    template <typename T>
    mesh::Vertex<T> QuadricErrorMetrics<T>::get_best_vertex(const Pair &pair)
    {
        // TODO
        return mesh::interpolation(0.5, mesh.vertices.at(pair.v1), mesh.vertices.at(pair.v2));
    };

    template <typename T>
    void QuadricErrorMetrics<T>::tidy_mesh()
    {
        // remove invalid vertices
        int i = 0;
        for (int j = 0; j < mesh.vertices.size(); j++)
        {
            if (vertexVersions[j] != INVALID)
            {
                for (auto faceID : vertexFaces[j])
                    for (auto &v : mesh.faces.at(faceID))
                        if (v == j)
                            v = i;

                mesh.vertices[i++] = mesh.vertices[j];
            }
        }
        mesh.vertices.resize(i);

        // remove degenerate triangles
        i = 0;
        for (int j = 0; j < mesh.faces.size(); j++)
            if (!mesh::hasDegenerate(mesh.faces[j]))
                mesh.faces[i++] = mesh.faces[j];

        mesh.faces.resize(i);
    }
}
