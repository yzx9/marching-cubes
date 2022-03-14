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

    struct PairContraction
    {
        int v1;
        int v2;
        int version; // invalid when any vertex change
        double quadricError;
        bool operator<(const PairContraction &c) const { return quadricError > c.quadricError; }
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
        std::priority_queue<PairContraction> edges;

        void build_edges();
        void collapse_edge(const PairContraction &edge);
        void update_quadric_error(PairContraction &edge);
        mesh::Vertex<T> get_best_vertex(const PairContraction &edge);
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

        build_edges();
    }

    template <typename T>
    void QuadricErrorMetrics<T>::simplify(int simplifyN)
    {
        while (simplifyN && !edges.empty())
        {
            auto e = edges.top();
            edges.pop();
            if (vertexVersions[e.v1] == INVALID ||
                vertexVersions[e.v2] == INVALID ||
                vertexVersions[e.v1] + vertexVersions[e.v2] != e.version)
                continue;

            collapse_edge(e);
            simplifyN--;
        }

        tidy_mesh();
    };

    template <typename T>
    void QuadricErrorMetrics<T>::build_edges()
    {
        const int n = mesh.faces.size() * 3;
        std::unordered_map<long, PairContraction> edgeMap;
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
                if (edgeMap.count(id) == 0)
                    edgeMap[id] = {v1 : v1, v2 : v2, version : vertexVersions[v1] + vertexVersions[v2]};
            }

            for (auto v : face)
                vertexFaces[v].emplace_back(i);
        }

        for (auto &[_, edge] : edgeMap)
        {
            update_quadric_error(edge);
            edges.push(edge);
        }
    }

    template <typename T>
    void QuadricErrorMetrics<T>::collapse_edge(const PairContraction &edge)
    {
        mesh.vertices[edge.v1] = get_best_vertex(edge);
        vertexVersions[edge.v1]++;
        vertexVersions[edge.v2] = INVALID;

        // merge faces from v2 to v1
        for (auto faceId : vertexFaces[edge.v2])
        {
            auto &face = mesh.faces.at(faceId);
            auto flag = true;
            for (auto &v : face)
            {
                if (v == edge.v1)
                    flag = false;

                if (v == edge.v2)
                    v = edge.v1;
            }

            if (flag)
                vertexFaces[edge.v1].emplace_back(faceId);
        }

        //  insert new edges
        for (auto faceId : vertexFaces[edge.v1])
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

                PairContraction edge{
                    v1 : v1,
                    v2 : v2,
                    version : vertexVersions[v1] + vertexVersions[v2],
                };
                update_quadric_error(edge);
                edges.emplace(edge);
            }
        }
    };

    template <typename T>
    void QuadricErrorMetrics<T>::update_quadric_error(PairContraction &edge)
    {
        edge.quadricError = std::rand(); // TODO
    };

    template <typename T>
    mesh::Vertex<T> QuadricErrorMetrics<T>::get_best_vertex(const PairContraction &edge)
    {
        // TODO
        return mesh::interpolation(0.5, mesh.vertices.at(edge.v1), mesh.vertices.at(edge.v2));
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
