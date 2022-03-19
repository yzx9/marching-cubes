#pragma once
#include <cmath>
#include <queue>
#include <tuple>
#include <vector>
#include <set>
#include <limits>
#include "Matrix.hpp"
#include "Mesh.hpp"
#include "Vec.hpp"

namespace quadric_error_metrics
{
    using matrix::SymmetryMatrix4;
    using mesh::Mesh;
    constexpr int INVALID = std::numeric_limits<int>::min();

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
        std::vector<std::set<int>> vertexFaces;
        std::vector<int> vertexVersions;
        std::priority_queue<Pair<T>> pairs;
        std::vector<SymmetryMatrix4<T>> faceKp;
        std::vector<SymmetryMatrix4<T>> verticeKp;
        std::vector<bool> validFaces;

        void build_pairs();
        void contract_pair(const Pair<T> &pair);
        void tidy_mesh();

        void update_face_kp(int faceID);
        void update_vertice_kp(int verticeID);
        void emplace_pair(int v1, int v2);
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
        : mesh(mesh),
          vertexFaces(mesh.vertices.size()),
          vertexVersions(mesh.vertices.size(), 1),
          faceKp(mesh.faces.size()),
          verticeKp(mesh.vertices.size()),
          validFaces(mesh.faces.size(), true)
    {
        // build vertex faces
        for (auto i = 0; i < mesh.faces.size(); i++)
            for (int j = 0; j < mesh.faces[i].size(); j++)
                vertexFaces[mesh.faces[i][j]].emplace(i);

        // build face Kp
        for (int i = 0; i < mesh.faces.size(); i++)
            update_face_kp(i);

        // build vertex Kp
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
            auto p = pairs.top();
            pairs.pop();
            if (vertexVersions[p.v1] + vertexVersions[p.v2] == p.version)
            {
                contract_pair(p);
                simplifyN--;
            }
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
            if (mesh::hasDegenerate(face))
            {
                validFaces[i] = false;
                continue;
            }

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

                pairIds.insert(id);
                emplace_pair(v1, v2);
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
            for (int i = 0; i < face.size(); i++)
            {
                if (face[i] == pair.v1)
                    validFaces[faceID] = false;

                if (face[i] == pair.v2)
                    face[i] = pair.v1;
            }

            if (validFaces[faceID])
                vertexFaces[pair.v1].emplace(faceID);
        }
        vertexFaces[pair.v2].clear();

        // update Kp
        for (auto faceID : vertexFaces[pair.v1])
            if (validFaces[faceID])
                update_face_kp(faceID);

        update_vertice_kp(pair.v1);

        //  insert new pairs
        for (auto faceID : vertexFaces[pair.v1])
        {
            const auto &face = mesh.faces.at(faceID);
            if (!validFaces[faceID])
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

                emplace_pair(v1, v2);
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
            if (validFaces[faceID])
                kp = kp + faceKp[faceID];
    }

    template <typename T>
    void QuadricErrorMetrics<T>::emplace_pair(int v1, int v2)
    {
        // TODO
        auto vertex = mesh::interpolate(0.5, mesh.vertices[v1], mesh.vertices[v2]);

        // Calc quadric error, Kp potentially contains planes(v1) ∩ planes(v2) twice.
        vec::Vec4<T> v(vertex.coord, 1);
        T quadricError = v * (verticeKp[v1] + verticeKp[v2]) * v;

        pairs.emplace(Pair<T>{
            v1 : v1,
            v2 : v2,
            version : vertexVersions[v1] + vertexVersions[v2],
            quadricError : quadricError,
            newVertex : vertex
        });
    }

    template <typename T>
    void QuadricErrorMetrics<T>::tidy_mesh()
    {
        // remove invalid vertices
        int i = 0;
        for (int j = 0; j < mesh.vertices.size(); j++)
        {
            if (vertexVersions[j] == INVALID)
                continue;

            for (auto faceID : vertexFaces[j])
                for (int k = 0; k < mesh.faces[faceID].size(); k++)
                    if (mesh.faces[faceID][k] == j)
                        mesh.faces[faceID][k] = i;

            mesh.vertices[i++] = std::move(mesh.vertices[j]);
        }
        mesh.vertices.resize(i);

        // remove degenerate triangles
        i = 0;
        for (int j = 0; j < mesh.faces.size(); j++)
            if (validFaces[j])
                mesh.faces[i++] = mesh.faces[j];

        mesh.faces.resize(i);
    }
}
