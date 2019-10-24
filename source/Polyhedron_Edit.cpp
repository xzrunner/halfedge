#include "halfedge/Polyhedron.h"
#include "halfedge/EditHelper.h"

#include <SM_Calc.h>

#include <map>

namespace
{

void BuildMapVert2Edges(const he::Polyhedron& src, std::vector<std::pair<he::Vertex*, std::vector<he::Edge*>>>& dst)
{
    std::map<he::Vertex*, size_t> vert2idx;

    auto vert_first = src.GetVertices().Head();
    auto vert_curr = vert_first;
    size_t idx = 0;
    do {
        dst.push_back({ vert_curr, std::vector<he::Edge*>() });
        vert2idx.insert({ vert_curr, idx++ });

        vert_curr = vert_curr->linked_next;
    } while (vert_curr != vert_first);

    auto edge_first = src.GetEdges().Head();
    auto edge_curr = edge_first;
    do {
        auto itr = vert2idx.find(edge_curr->vert);
        assert(itr != vert2idx.end());
        dst[itr->second].second.push_back(edge_curr);
        edge_curr = edge_curr->linked_next;
    } while (edge_curr != edge_first);
}

void BuildMapVert2Planes(const he::Polyhedron& src, std::vector<sm::Plane>& planes,
                         std::map<he::Vertex*, std::vector<size_t>>& dst)
{
    auto first_face = src.GetFaces().Head();
    auto curr_face = first_face;
    do {
        sm::Plane plane;
        he::face_to_plane(*curr_face, plane);
        planes.push_back(plane);

        curr_face = curr_face->linked_next;
    } while (curr_face != first_face);

    size_t plane_idx = 0;
    first_face = src.GetFaces().Head();
    curr_face = first_face;
    do {
        auto first_edge = curr_face->edge;
        auto curr_edge = first_edge;
        do {
            auto itr = dst.find(curr_edge->vert);
            if (itr == dst.end()) {
                dst.insert({ curr_edge->vert, { plane_idx } });
            } else {
                itr->second.push_back(plane_idx);
            }

            curr_edge = curr_edge->next;
        } while (curr_edge != first_edge);

        ++plane_idx;
        curr_face = curr_face->linked_next;
    } while (curr_face != first_face);
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
    std::vector<std::pair<he::Vertex*, std::vector<he::Edge*>>> vert2edges;
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

void Polyhedron::UniquePoints()
{
    EditHelper::UniquePoints(m_vertices, m_edges, m_next_vert_id);
}

bool Polyhedron::Extrude(float distance, const std::vector<TopoID>& face_ids, bool create_face[ExtrudeMaxCount],
                         std::vector<Face*>* new_faces)
{
    if (distance == 0) {
        return false;
    }

    const bool add_front = create_face[ExtrudeFront];
    const bool add_back  = create_face[ExtrudeBack];
    const bool add_side  = create_face[ExtrudeSide];

    std::vector<sm::Plane> planes;
    std::map<he::Vertex*, std::vector<size_t>> vert2planes;
    BuildMapVert2Planes(*this, planes, vert2planes);

    std::vector<Face*> old_front_faces;

    std::vector<Vertex*> new_vts;
    std::map<Vertex*, Vertex*> vert_old2new;

    std::vector<Face*> new_front_faces;
    std::vector<std::vector<Edge*>> new_front_edges;

    size_t plane_idx = 0;
    auto first_face = m_faces.Head();
    auto curr_face = first_face;
    do {
        bool find = false;
        for (auto& id : face_ids) {
            if (id == curr_face->ids) {
                find = true;
                break;
            }
        }

        if (find)
        {
            planes[plane_idx].dist -= distance;

            old_front_faces.push_back(curr_face);

            auto new_face = new Face(m_next_face_id++);
            new_front_faces.push_back(new_face);

            std::vector<Edge*> new_edges;

            auto first_edge = curr_face->edge;
            auto curr_edge = first_edge;
            do {
                Vertex* new_v = nullptr;
                auto old_v = curr_edge->vert;
                auto itr = vert_old2new.find(old_v);
                if (itr == vert_old2new.end())
                {
                    new_v = new Vertex(old_v->position, m_next_vert_id++);
                    new_vts.push_back(new_v);
                    vert_old2new.insert({ old_v, new_v });
                }
                else
                {
                    new_v = itr->second;
                }

                auto new_edge = new Edge(new_v, new_face, m_next_edge_id++);
                new_edges.push_back(new_edge);

                curr_edge = curr_edge->next;
            } while (curr_edge != first_edge);

            new_front_edges.push_back(new_edges);

            assert(!new_edges.empty());
            new_face->edge = new_edges.front();
        }

        ++plane_idx;
        curr_face = curr_face->linked_next;
    } while (curr_face != first_face);

    assert(old_front_faces.size() == new_front_faces.size()
        && new_front_faces.size() == new_front_edges.size());

    // connect edges
    for (auto& edges : new_front_edges) {
        for (int i = 0, n = edges.size(); i < n; ++i) {
            edges[i]->Connect(edges[(i + 1) % n]);
        }
    }

    // offset vertices
    for (auto itr : vert_old2new)
    {
        auto itr_planes = vert2planes.find(itr.first);
        assert(itr_planes != vert2planes.end());
        auto& p_ids = itr_planes->second;
        if (p_ids.size() < 3)
        {
            assert(!p_ids.empty());
            auto& p0 = planes[p_ids[0]];
            auto edge = itr.first->edge;
            auto prev_edge = edge->prev;
            assert(edge && prev_edge);
            if (p_ids.size() == 1)
            {
                auto p1 = sm::Plane(p0.normal.Cross(edge->next->vert->position - edge->vert->position), itr.first->position);
                auto p2 = sm::Plane(p0.normal.Cross(prev_edge->next->vert->position - prev_edge->vert->position), itr.first->position);
                bool intersect = sm::intersect_planes(p0, p1, p2, &itr.second->position);
                assert(intersect);
            }
            else
            {
                assert(p_ids.size() == 2);
                auto& p1 = planes[p_ids[1]];
                auto p2 = sm::Plane(p0.normal.Cross(edge->next->vert->position - edge->vert->position), itr.first->position);
                if (!sm::intersect_planes(p0, p1, p2, &itr.second->position)) {
                    p2 = sm::Plane(p0.normal.Cross(prev_edge->next->vert->position - prev_edge->vert->position), itr.first->position);
                    bool intersect = sm::intersect_planes(p0, p1, p2, &itr.second->position);
                    assert(intersect);
                }
            }
        }
        else
        {
            auto& p0 = planes[p_ids[0]];
            auto& p1 = planes[p_ids[1]];
            auto& p2 = planes[p_ids[2]];
            bool intersect = sm::intersect_planes(p0, p1, p2, &itr.second->position);
            assert(intersect);
        }
    }

    // use new vts
    if (add_front || add_side) {
        for (auto& v : new_vts) {
            m_vertices.Append(v);
        }
    } else {
        for (auto& v : new_vts) {
            delete v;
        }
    }
    // use new front faces
    if (add_front) {
        for (auto& f : new_front_faces) {
            m_faces.Append(f);
        }
    } else {
        for (auto& f : new_front_faces) {
            delete f;
        }
    }
    // use new front edges
    if (add_front || add_side) {
        for (auto& edges : new_front_edges) {
            for (auto& e : edges) {
                m_edges.Append(e);
            }
        }
    } else {
        for (auto& edges : new_front_edges) {
            for (auto& e : edges) {
                delete e;
            }
        }
    }

    // create side faces
    std::vector<std::vector<std::vector<Edge*>>> new_side_edges;
    std::vector<std::vector<Face*>> new_side_faces;
    if (add_side)
    {
        assert(old_front_faces.size() == new_front_faces.size()
            && new_front_faces.size() == new_front_edges.size());

        // create side faces
        new_side_faces.resize(new_front_edges.size());
        for (size_t i = 0, n = new_front_edges.size(); i < n; ++i) {
            new_side_faces[i].resize(new_front_edges[i].size());
            for (size_t j = 0, m = new_front_edges[i].size(); j < m; ++j) {
                new_side_faces[i][j] = new Face(m_next_face_id++);
            }
        }

        // prepare side verts
        std::vector<std::vector<Vertex*>> old_face_vts, new_face_vts;
        old_face_vts.resize(old_front_faces.size());
        new_face_vts.resize(new_front_faces.size());
        assert(old_face_vts.size() == new_face_vts.size());
        for (size_t i = 0, n = old_front_faces.size(); i < n; ++i)
        {
            auto first_edge = old_front_faces[i]->edge;
            auto curr_edge = first_edge;
            do {
                old_face_vts[i].push_back(curr_edge->vert);
                curr_edge = curr_edge->next;
            } while (curr_edge != first_edge);

            for (auto& e : new_front_edges[i]) {
                new_face_vts[i].push_back(e->vert);
            }

            assert(old_face_vts[i].size() == new_face_vts[i].size());
        }

        // create side edges
        new_side_edges.resize(new_front_edges.size());
        for (size_t i = 0, n = new_front_edges.size(); i < n; ++i)
        {
            new_side_edges[i].resize(new_front_edges[i].size());
            for (size_t j = 0, m = new_front_edges[i].size(); j < m; ++j)
            {
                new_side_edges[i][j].resize(4);
                new_side_edges[i][j][0] = new Edge(old_face_vts[i][j], new_side_faces[i][j], m_next_edge_id++);
                new_side_edges[i][j][1] = new Edge(old_face_vts[i][(j + 1) % old_face_vts[i].size()], new_side_faces[i][j], m_next_edge_id++);
                new_side_edges[i][j][2] = new Edge(new_face_vts[i][(j + 1) % new_face_vts[i].size()], new_side_faces[i][j], m_next_edge_id++);
                new_side_edges[i][j][3] = new Edge(new_face_vts[i][j], new_side_faces[i][j], m_next_edge_id++);
                for (size_t k = 0; k < 4; ++k) {
                    new_side_edges[i][j][k]->Connect(new_side_edges[i][j][(k + 1) % new_side_edges[i][j].size()]);
                }
                new_side_faces[i][j]->edge = new_side_edges[i][j][0];
            }
        }

        // append side faces and edges
        std::set<Edge*> all_old_edges;
        for (auto& f : old_front_faces) {
            auto first_edge = f->edge;
            auto curr_edge = first_edge;
            do {
                all_old_edges.insert(curr_edge);
                curr_edge = curr_edge->next;
            } while (curr_edge != first_edge);
        }

        // prepare for side seam
        std::map<Edge*, std::pair<size_t, size_t>> old_front_edge_to_idx;
        for (size_t i = 0, n = old_front_faces.size(); i < n; ++i)
        {
            size_t j = 0;
            auto first_edge = old_front_faces[i]->edge;
            auto curr_edge = first_edge;
            do {
                old_front_edge_to_idx.insert({ curr_edge, { i, j } });
                ++j;
                curr_edge = curr_edge->next;
            } while (curr_edge != first_edge);
        }

        for (size_t i = 0, n = old_front_faces.size(); i < n; ++i)
        {
            auto old_f = old_front_faces[i];
            auto first_edge = old_f->edge;
            auto curr_edge = first_edge;
            size_t edge_idx = 0;
            do {
                if (curr_edge->twin && all_old_edges.find(curr_edge->twin) != all_old_edges.end())
                {
                    delete new_side_faces[i][edge_idx];
                    new_side_faces[i][edge_idx] = nullptr;
                    for (auto& e : new_side_edges[i][edge_idx]) {
                        delete e;
                        e = nullptr;
                    }
                }
                else
                {
                    m_faces.Append(new_side_faces[i][edge_idx]);
                    for (auto& e : new_side_edges[i][edge_idx]) {
                        m_edges.Append(e);
                    }

                    // find twin
                    auto next_old_front_edge = curr_edge->next;
                    while (next_old_front_edge->twin && all_old_edges.find(next_old_front_edge->twin) != all_old_edges.end()) {
                        next_old_front_edge = next_old_front_edge->twin->next;
                    }
                    auto itr = old_front_edge_to_idx.find(next_old_front_edge);
                    assert(itr != old_front_edge_to_idx.end());
                    edge_make_pair(new_side_edges[i][edge_idx][1], new_side_edges[itr->second.first][itr->second.second][3]);
                }

                ++edge_idx;
                curr_edge = curr_edge->next;
            } while (curr_edge != first_edge);
        }
    }

    // create back faces
    std::vector<Face*> new_back_faces;
    std::vector<std::vector<Edge*>> new_back_edges;
    new_back_edges.resize(old_front_faces.size());
    if (add_back)
    {
        for (size_t i = 0, n = old_front_faces.size(); i < n; ++i)
        {
            auto& old_f = old_front_faces[i];

            auto new_f = new Face(m_next_face_id++);
            new_back_faces.push_back(new_f);

            auto first_edge = old_f->edge;
            auto curr_edge = first_edge;
            do {
                auto new_edge = new Edge(curr_edge->vert, new_f, m_next_edge_id++);
                new_back_edges[i].push_back(new_edge);
                m_edges.Append(new_edge);

                curr_edge = curr_edge->next;
            } while (curr_edge != first_edge);

            assert(new_back_edges[i].size() > 1);
            for (int j = 0, m = new_back_edges[i].size(); j < m; ++j) {
                new_back_edges[i][j]->Connect(new_back_edges[i][(j + m - 1) % m]);
            }

            new_f->edge = new_back_edges[i].front();

            m_faces.Append(new_f);
        }
    }

    // seam
    if (add_side && (add_front || add_back))
    {
        for (size_t i = 0, n = new_side_faces.size(); i < n; ++i) {
            for (size_t j = 0, m = new_side_faces[i].size(); j < m; ++j) {
                if (!new_side_faces[i][j]) {
                    continue;
                }
                if (add_front)
                {
                    auto side = new_side_edges[i][j][2];
                    auto front = new_front_edges[i][j];
                    assert(side && front);
                    edge_make_pair(side, front);
                }
                if (add_back)
                {
                    auto side = new_side_edges[i][j][0];
                    auto back = new_back_edges[i][(j + 1) % new_back_edges[i].size()];
                    assert(side && back);
                    edge_make_pair(side, back);
                }
            }
        }
    }

    // rm old faces
    for (auto& f : old_front_faces) {
        RemoveFace(f);
    }

    UpdateAABB();

    // return
    if (new_faces)
    {
        new_faces[ExtrudeFront] = new_front_faces;
        new_faces[ExtrudeBack]  = new_back_faces;

        new_faces[ExtrudeSide].clear();
        for (auto& faces : new_side_faces) {
            for (auto& face : faces) {
                new_faces[ExtrudeSide].push_back(face);
            }
        }
    }

    return true;
}

void Polyhedron::RemoveFace(Face* face)
{
    m_faces.Remove(face);

    std::vector<std::pair<he::Vertex*, std::vector<he::Edge*>>> vert2edges;
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