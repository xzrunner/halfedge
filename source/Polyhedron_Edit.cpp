#include "halfedge/Polyhedron.h"

#include <SM_Calc.h>

#include <map>

namespace
{

void BuildMapVert2Edges(const he::Polyhedron& src, std::map<he::Vertex*, std::vector<he::Edge*>>& dst)
{
    auto vert_first = src.GetVertices().Head();
    auto vert_curr = vert_first;
    do {
        auto ret = dst.insert({ vert_curr, std::vector<he::Edge*>() });
        assert(ret.second);
        vert_curr = vert_curr->linked_next;
    } while (vert_curr != vert_first);

    auto edge_first = src.GetEdges().Head();
    auto edge_curr = edge_first;
    do {
        auto itr = dst.find(edge_curr->vert);
        assert(itr != dst.end());
        itr->second.push_back(edge_curr);
        edge_curr = edge_curr->linked_next;
    } while (edge_curr != edge_first);
}

}

namespace he
{

void Polyhedron::Fill()
{
    std::map<Vertex*, Edge*> new_edges;

    // create edges
    auto first = m_edges.Head();
    auto curr = first;
    do {
        if (!curr->twin)
        {
            auto edge = new Edge(curr->next->vert, nullptr, m_next_edge_id++);
            auto ret = new_edges.insert({ edge->vert, edge });
            assert(ret.second);
            edge_make_pair(edge, curr);
            m_edges.Append(edge);
        }
        curr = curr->linked_next;
    } while (curr != first);

    // create face and connect new edges
    for (auto itr = new_edges.begin(); itr != new_edges.end(); ++itr)
    {
        Edge* edge = itr->second;
        if (edge->face) {
            continue;
        }

        auto face = new Face(m_next_face_id++);
        face->edge = edge;

        Vertex* first = edge->vert;
        do {
            edge->face = face;

            auto itr = new_edges.find(edge->twin->vert);
            assert(itr != new_edges.end());
            auto next_edge = itr->second;

            edge->Connect(next_edge);
            edge = next_edge;
        } while (edge->vert != first);

        m_faces.Append(face);
    }
}

void Polyhedron::Fuse(float distance)
{
    std::map<Vertex*, std::vector<Edge*>> vert2edges;
    BuildMapVert2Edges(*this, vert2edges);

    for (auto itr0 = vert2edges.begin(); itr0 != vert2edges.end(); ++itr0)
    {
        if (!itr0->first->ids.IsValid()) {
            continue;
        }

        auto itr1 = itr0;
        ++itr1;
        for (; itr1 != vert2edges.end(); ++itr1)
        {
            if (!itr1->first->ids.IsValid()) {
                continue;
            }

            auto d = sm::dis_pos3_to_pos3(
                itr0->first->position, itr1->first->position
            );
            if (d >= distance) {
                continue;
            }

            for (auto& edge : itr1->second) {
                edge->vert = itr0->first;
            }
            itr1->first->ids.MakeInvalid();
            m_vertices.Remove(itr1->first);
        }
    }

    for (auto itr : vert2edges) {
        if (!itr.first->ids.IsValid()) {
            delete itr.first;
        }
    }
}

PolyhedronPtr Polyhedron::Fuse(const std::vector<PolyhedronPtr>& polys, float distance)
{
    std::vector<std::pair<TopoID, sm::vec3>> vertices;
    std::vector<std::pair<TopoID, std::vector<size_t>>> faces;

    std::map<size_t, size_t> vert_uid2pos;
    for (auto& poly : polys)
    {
        auto first_vert = poly->GetVertices().Head();
        auto curr_vert = first_vert;
        do {
            auto ret = vert_uid2pos.insert({ curr_vert->ids.UID(), vertices.size() });
            assert(ret.second);
            vertices.push_back({ curr_vert->ids, curr_vert->position });
            curr_vert = curr_vert->linked_next;
        } while (curr_vert != first_vert);
    }

    for (auto& poly : polys)
    {
        auto first_face = poly->GetFaces().Head();
        auto curr_face = first_face;
        do {
            std::vector<size_t> face;

            auto first_edge = curr_face->edge;
            auto curr_edge = first_edge;
            do {
                auto itr = vert_uid2pos.find(curr_edge->vert->ids.UID());
                assert(itr != vert_uid2pos.end());
                face.push_back(itr->second);

                curr_edge = curr_edge->next;
            } while (curr_edge != first_edge);

            faces.push_back({ curr_face->ids, face });

            curr_face = curr_face->linked_next;
        } while (curr_face != first_face);
    }

    auto ret = std::make_shared<Polyhedron>(vertices, faces);
    ret->Fuse(distance);
    return ret;
}

}