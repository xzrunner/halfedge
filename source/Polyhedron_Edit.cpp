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

    UpdateAABB();
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

bool Polyhedron::Extrude(float distance, TopoID face_id, bool add_front, bool add_back, bool add_side)
{
    if (distance == 0) {
        return false;
    }

    Face* face = nullptr;
    auto first_face = m_faces.Head();
    auto curr_face = first_face;
    do {
        if (curr_face->ids == face_id) {
            face = curr_face;
            break;
        }

        curr_face = curr_face->linked_next;
    } while (curr_face != first_face);

    if (!face) {
        return false;
    }

    if (face->edge->twin && !face->edge->next->twin) {
        int zz = 0;
    }

    sm::Plane plane;
    face_to_plane(*face, plane);
    auto offset = plane.normal * distance;

    // create front face
    Face* front_face = nullptr;
    if (add_front)
    {
        auto new_face = new Face(face->ids);
        m_faces.Append(new_face);

        auto first_edge = face->edge;
        auto curr_edge = first_edge;
        Edge* prev_new_edge = nullptr;
        Edge* first_new_edge = nullptr;
        Edge* last_new_edge = nullptr;
        do {
            auto new_vert = new Vertex(curr_edge->vert->position + offset, m_next_vert_id++);
            m_vertices.Append(new_vert);
            auto new_edge = new Edge(new_vert, new_face, m_next_edge_id++);
            m_edges.Append(new_edge);
            if (!prev_new_edge) {
                assert(!first_new_edge);
                prev_new_edge = new_edge;
                first_new_edge = new_edge;
            } else {
                prev_new_edge->Connect(new_edge);
                prev_new_edge = new_edge;
            }
            last_new_edge = new_edge;

            curr_edge = curr_edge->next;
        } while (curr_edge != first_edge);

        assert(last_new_edge && first_new_edge);
        last_new_edge->Connect(first_new_edge);

        new_face->edge = first_new_edge;

        front_face = new_face;
    }

    // create back face
    Face* back_face = nullptr;
    if (add_back)
    {
        auto new_face = new Face(m_next_face_id++);
        m_faces.Append(new_face);

        auto first_edge = face->edge;
        auto curr_edge = first_edge;
        Edge* prev_new_edge = nullptr;
        Edge* first_new_edge = nullptr;
        Edge* last_new_edge = nullptr;
        do {
            auto new_edge = new Edge(curr_edge->vert, new_face, m_next_edge_id++);
            m_edges.Append(new_edge);
            if (!prev_new_edge) {
                assert(!first_new_edge);
                prev_new_edge = new_edge;
                first_new_edge = new_edge;
            } else {
                prev_new_edge->Connect(new_edge);
                prev_new_edge = new_edge;
            }
            last_new_edge = new_edge;

            curr_edge = curr_edge->prev;
        } while (curr_edge != first_edge);

        assert(last_new_edge && first_new_edge);
        last_new_edge->Connect(first_new_edge);

        new_face->edge = first_new_edge;

        back_face = new_face;
    }

    // create side faces
    std::vector<Face*> side_faces;
    if (add_side)
    {
        // prepare vertices
        std::vector<Vertex*> old_vts;
        auto first_edge = face->edge;
        auto curr_edge = first_edge;
        do {
            old_vts.push_back(curr_edge->vert);
            curr_edge = curr_edge->next;
        } while (curr_edge != first_edge);
        assert(old_vts.size() > 1);

        std::vector<Vertex*> new_vts;
        if (front_face)
        {
            auto first_edge = front_face->edge;
            auto curr_edge = first_edge;
            do {
                new_vts.push_back(curr_edge->vert);
                curr_edge = curr_edge->next;
            } while (curr_edge != first_edge);
        }
        else
        {
            for (auto& old_v : old_vts) {
                auto new_v = new Vertex(old_v->position + offset, m_next_vert_id++);
                m_vertices.Append(new_v);
                new_vts.push_back(new_v);
            }
        }
        assert(old_vts.size() == new_vts.size());

        // create edges and faces
        for (int i = 0, n = old_vts.size(); i < n; ++i)
        {
            const int next_idx = (i + 1) % n;

            auto new_face = new Face(m_next_face_id++);
            m_faces.Append(new_face);
            side_faces.push_back(new_face);

            auto e0 = new Edge(old_vts[i], new_face, m_next_edge_id++);
            auto e1 = new Edge(old_vts[next_idx], new_face, m_next_edge_id++);
            auto e2 = new Edge(new_vts[next_idx], new_face, m_next_edge_id++);
            auto e3 = new Edge(new_vts[i], new_face, m_next_edge_id++);
            e0->Connect(e1)->Connect(e2)->Connect(e3)->Connect(e0);

            new_face->edge = e0;
        }

        // seam
        for (int i = 0, n = side_faces.size(); i < n; ++i)
        {
            auto curr = side_faces[i];
            auto next = side_faces[(i + 1) % n];
            edge_make_pair(curr->edge->next, next->edge->next->next->next);
        }
    }

    // seam
    if (add_side)
    {
        if (add_front)
        {
            assert(side_faces.size() > 0 && front_face);
            auto front_edge = front_face->edge;
            for (int i = 0, n = side_faces.size(); i < n; ++i) {
                edge_make_pair(side_faces[i]->edge->next->next, front_edge);
                front_edge = front_edge->next;
            }
        }
        if (add_back)
        {
            assert(side_faces.size() > 0 && back_face);
            auto back_edge = back_face->edge->prev;
            for (int i = 0, n = side_faces.size(); i < n; ++i) {
                edge_make_pair(side_faces[i]->edge->next->next, back_edge);
                back_edge = back_edge->prev;
            }
        }
        else
        {
            auto old_edge = face->edge;
            for (int i = 0, n = side_faces.size(); i < n; ++i) {
                if (old_edge->twin) {
                    edge_make_pair(side_faces[i]->edge, old_edge->twin);
                }
                old_edge = old_edge->next;
            }
        }
    }

    // rm old face
    RemoveFace(face);

    UpdateAABB();

    return true;
}

void Polyhedron::RemoveFace(Face* face)
{
    m_faces.Remove(face);

    std::map<Vertex*, std::vector<Edge*>> vert2edges;
    BuildMapVert2Edges(*this, vert2edges);

    std::vector<Edge*> del_edges;
    auto first_edge = face->edge;
    auto curr_edge = first_edge;
    do {
        del_edges.push_back(curr_edge);
        curr_edge->ids.MakeInvalid();
        if (curr_edge->twin && curr_edge->twin->twin == curr_edge) {
            curr_edge->twin->twin = nullptr;
        }
        m_edges.Remove(curr_edge);

        curr_edge = curr_edge->next;
    } while (curr_edge != first_edge);

    for (auto& itr : vert2edges)
    {
        auto& v = itr.first;
        if (v->edge->ids.IsValid()) {
            continue;
        }

        for (auto& e : itr.second) {
            if (e->ids.IsValid()) {
                v->edge = e;
                break;
            }
        }

        if (!v->edge->ids.IsValid()) {
            m_vertices.Remove(v);
            delete v;
        }
    }

    for (auto& e : del_edges) {
        delete e;
    }
    delete face;
}

}