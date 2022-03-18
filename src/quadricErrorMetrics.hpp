#pragma once
#include <cmath>
#include <queue>
#include <tuple>
#include <vector>
#include <set>
#include "Matrix.hpp"
#include "Mesh.hpp"
#include "Vec.hpp"

namespace quadric_error_metrics
{
    using matrix::SymmetryMatrix4;
    using mesh::Mesh;
    constexpr int INVALID = 0;

    template <typename T>
    struct Pair
    {
        int v1;
        int v2;
        int version; // invalid when any vertex change
        T quadricError;
        mesh::Vertex<T> newVertex;
        bool operator<(const Pair<T> &p) const { return quadricError >= p.quadricError; }
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
        std::priority_queue<Pair<T>> pairs;
        std::vector<SymmetryMatrix4<T>> faceKp;
        std::vector<SymmetryMatrix4<T>> verticeKp;

        void build_pairs();
        void contract_pair(const Pair<T> &pair);
        void tidy_mesh();

        void update_face_kp(int faceID);
        void update_vertice_kp(int verticeID);
        Pair<T> new_pair(int v1, int v2) const;
    };

    template <typename T>
    void simplify(Mesh<T> &mesh, double simplifyPercent)
    {
        const auto simplifyN = std::ceil(mesh.vertices.size() * simplifyPercent);
        QuadricErrorMetrics<T> qem(mesh);
        qem.simplify(simplifyN);
    };

    template <typename T>
    QuadricErrorMetrics<T>::QuadricErrorMetrics(Mesh<T> &mesh)
        : mesh(mesh), vertexFaces(mesh.vertices.size()), vertexVersions(mesh.vertices.size(), INVALID + 1)
    {
        // build vertex faces
        for (auto i = 0; i < mesh.faces.size(); i++)
            for (int j = 0; j < mesh.faces[i].size(); j++)
                vertexFaces[mesh.faces[i][j]].emplace_back(i);

        // build face Kp
        faceKp.resize(mesh.faces.size());
        for (int i = 0; i < mesh.faces.size(); i++)
            update_face_kp(i);

        // build vertex Kp
        verticeKp.resize(mesh.vertices.size());
        for (int i = 0; i < mesh.vertices.size(); i++)
            update_vertice_kp(i);

        // build vertex pairs
        build_pairs();
    }

    template <typename T>
    void QuadricErrorMetrics<T>::simplify(int simplifyN)
    {
        while (simplifyN && !pairs.empty())
        {
            const auto &p = pairs.top();
            if (vertexVersions[p.v1] + vertexVersions[p.v2] == p.version)
            {
                contract_pair(p);
                simplifyN--;
            }

            pairs.pop();
        }

        tidy_mesh();
    };

    template <typename T>
    void QuadricErrorMetrics<T>::build_pairs()
    {
        std::set<long> pairIds;
        for (auto i = 0; i < mesh.faces.size(); i++)
        {
            const auto &face = mesh.faces[i];
            std::array<int, 6> faceEdges{face[0], face[1],
                                         face[1], face[2],
                                         face[2], face[0]};
            for (auto j = 0; j < faceEdges.size(); j += 2)
            {
                auto v1 = faceEdges[j + 0];
                auto v2 = faceEdges[j + 1];
                if (v1 > v2)
                    std::swap(v1, v2);

                long id = v1 << 32 + v2;
                if (pairIds.contains(id))
                    continue;

                auto pair = new_pair(v1, v2);
                pairs.push(pair);
                pairIds.insert(id);
            }
        }
    }

    template <typename T>
    void QuadricErrorMetrics<T>::contract_pair(const Pair<T> &pair)
    {
        mesh.vertices[pair.v1] = pair.newVertex;
        vertexVersions[pair.v1]++;
        vertexVersions[pair.v2] = INVALID;

        // merge faces from v2 to v1
        for (auto faceID : vertexFaces[pair.v2])
        {
            auto &face = mesh.faces.at(faceID);
            auto flag = true;
            for (int i = 0; i < face.size(); i++)
            {
                if (face[i] == pair.v1)
                    flag = false; // degenerate face

                if (face[i] == pair.v2)
                    face[i] = pair.v1;
            }

            if (flag)
                vertexFaces[pair.v1].emplace_back(faceID);
        }

        // update kp
        for (auto faceID : vertexFaces[pair.v1])
            if (!mesh::hasDegenerate(mesh.faces.at(faceID)))
                update_face_kp(faceID);

        update_vertice_kp(pair.v1);

        //  insert new pairs
        for (auto faceID : vertexFaces[pair.v1])
        {
            const auto &face = mesh.faces.at(faceID);
            if (mesh::hasDegenerate(face))
                continue;

            std::array<int, 6> faceEdges{face[0], face[1],
                                         face[1], face[2],
                                         face[2], face[0]};
            for (auto j = 0; j < faceEdges.size(); j += 2)
            {
                auto v1 = faceEdges[j + 0];
                auto v2 = faceEdges[j + 1];
                if (v1 != pair.v1 && v2 != pair.v1)
                    continue;

                if (v1 > v2)
                    std::swap(v1, v2);

                pairs.emplace(new_pair(v1, v2));
            }
        }
    };

    template <typename T>
    void QuadricErrorMetrics<T>::update_face_kp(int faceID)
    {
        const auto &v = mesh.vertices;
        const auto &f = mesh.faces[faceID];
        const auto &v0 = v[f[0]].coord;
        auto normal = vec::normalize(vec::product(v0 - v[f[1]].coord, v0 - v[f[2]].coord));

        auto a = normal[0];
        auto b = normal[1];
        auto c = normal[2];
        auto d = -a * v0[0] - b * v0[1] - c * v0[2];
        faceKp[faceID] = SymmetryMatrix4(a * a, a * b, a * c, a * d,
                                         /*  */ b * b, b * c, b * d,
                                         /*         */ c * c, c * d,
                                         /*                */ d * d);
    }

    template <typename T>
    void QuadricErrorMetrics<T>::update_vertice_kp(int verticeID)
    {
        SymmetryMatrix4<T> kp;
        kp.fill(static_cast<T>(0));
        for (auto faceID : vertexFaces[verticeID])
            if (!mesh::hasDegenerate(mesh.faces[faceID]))
                kp = kp + faceKp[faceID];
    }

    template <typename T>
    Pair<T> QuadricErrorMetrics<T>::new_pair(int v1, int v2) const
    {
        // TODO
        auto vertex = mesh::interpolate(0.5, mesh.vertices[v1], mesh.vertices[v2]);

        // Calc quadric error, Kp potentially contains planes(v1) ∩ planes(v2) twice.
        vec::Vec4<T> v(vertex.coord, 1);
        T quadricError = v * (verticeKp[v1] + verticeKp[v2]) * v;

        return Pair<T>{
            v1 : v1,
            v2 : v2,
            version : vertexVersions[v1] + vertexVersions[v2],
            quadricError : quadricError,
            newVertex : vertex
        };
    }

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
                    for (int k = 0; k < mesh.faces[faceID].size(); k++)
                        if (mesh.faces[faceID][k] == j)
                            mesh.faces[faceID][k] = i;

                mesh.vertices[i++] = std::move(mesh.vertices[j]);
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
