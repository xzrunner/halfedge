#include "halfedge/Polyhedron.h"
#include "halfedge/Utility.h"

#include <SM_Calc.h>

#include <map>

namespace
{

void BuildMapVert2Edges(const he::Polyhedron& src, std::vector<std::pair<he::vert3*, std::vector<he::edge3*>>>& dst)
{
    std::map<he::vert3*, size_t> vert2idx;

    auto vert_first = src.GetVerts().Head();
    auto vert_curr = vert_first;
    size_t idx = 0;
    do {
        dst.push_back({ vert_curr, std::vector<he::edge3*>() });
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

void BuildMapVert2Planes(const he::loop3& loop, size_t plane_idx, std::map<he::vert3*, std::vector<size_t>>& dst)
{
    auto first_e = loop.edge;
    auto curr_e = first_e;
    do {
        auto itr = dst.find(curr_e->vert);
        if (itr == dst.end()) {
            dst.insert({ curr_e->vert, { plane_idx } });
        } else {
            itr->second.push_back(plane_idx);
        }

        curr_e = curr_e->next;
    } while (curr_e != first_e);
}

void RemoveLoop(he::Polyhedron& poly, he::loop3* loop)
{
    const_cast<he::DoublyLinkedList<he::loop3>&>(poly.GetLoops()).Remove(loop);

    std::vector<std::pair<he::vert3*, std::vector<he::edge3*>>> vert2edges;
    BuildMapVert2Edges(poly, vert2edges);

    std::vector<he::edge3*> del_edges;
    auto first_edge = loop->edge;
    auto curr_edge = first_edge;
    do {
        del_edges.push_back(curr_edge);
        curr_edge->ids.MakeInvalid();
        if (curr_edge->twin && curr_edge->twin->twin == curr_edge) {
            curr_edge->twin->twin = nullptr;
        }
        const_cast<he::DoublyLinkedList<he::edge3>&>(poly.GetEdges()).Remove(curr_edge);

        curr_edge = curr_edge->next;
    } while (curr_edge != first_edge);

    for (auto& itr : vert2edges)
    {
        auto& v = itr.first;
        assert(v->edge);
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
            const_cast<he::DoublyLinkedList<he::vert3>&>(poly.GetVerts()).Remove(v);
            delete v;
        }
    }

    for (auto& e : del_edges) {
        delete e;
    }
    delete loop;
}

void RemoveFace(he::Polyhedron& poly, const he::Polyhedron::Face& face)
{
    RemoveLoop(poly, face.border);
    for (auto& hole : face.holes) {
        RemoveLoop(poly, hole);
    }

    auto& faces = const_cast<std::vector<he::Polyhedron::Face>&>(poly.GetFaces());
    for (auto itr = faces.begin(); itr != faces.end(); ++itr) {
        if (itr->border == face.border) {
            faces.erase(itr);
            break;
        }
    }
}

he::loop3* CloneLoop(he::loop3* loop, std::map<he::vert3*, he::vert3*>& vert_old2new, std::vector<he::vert3*>& new_vts,
                     size_t& next_vert_id, size_t& next_edge_id, size_t& next_loop_id)
{
    auto new_face = new he::loop3(next_loop_id++);

    auto first_edge = loop->edge;
    auto curr_edge = first_edge;
    he::edge3* last_edge = nullptr;
    do {
        he::vert3* new_v = nullptr;
        auto old_v = curr_edge->vert;
        auto itr = vert_old2new.find(old_v);
        if (itr == vert_old2new.end())
        {
            new_v = new he::vert3(old_v->position, next_vert_id++);
            new_vts.push_back(new_v);
            vert_old2new.insert({ old_v, new_v });
        }
        else
        {
            new_v = itr->second;
        }

        auto new_edge = new he::edge3(new_v, new_face, next_edge_id++);

        if (!new_face->edge) {
            new_face->edge = new_edge;
        }

        if (!last_edge) {
            last_edge = new_edge;
        } else {
            last_edge->Connect(new_edge);
        }
        last_edge = new_edge;

        curr_edge = curr_edge->next;
    } while (curr_edge != first_edge);

    assert(last_edge && new_face->edge);
    last_edge->Connect(new_face->edge);

    return new_face;
}

void AppendEdges(const he::loop3& loop, he::DoublyLinkedList<he::edge3>& edges)
{
    auto first_e = loop.edge;
    auto curr_e = first_e;
    do {
        edges.Append(curr_e);

        curr_e = curr_e->next;
    } while (curr_e != first_e);
}

void DeleteEdges(const he::loop3& loop)
{
    std::vector<he::edge3*> edges;

    auto first_e = loop.edge;
    auto curr_e = first_e;
    do {
        edges.push_back(curr_e);

        curr_e = curr_e->next;
    } while (curr_e != first_e);

    for (auto& e : edges) {
        delete e;
    }
}

void CreateSideFaces(const he::loop3& old_loop, const he::loop3& new_loop,
                     std::vector<he::loop3*>& side_faces, size_t& next_loop_id, size_t& next_edge_id)
{
    // check
    size_t num = he::Utility::EdgeSize(old_loop);
    assert(num == he::Utility::EdgeSize(new_loop));

    // create side faces
    for (size_t i = 0; i < num; ++i) {
        side_faces.push_back(new he::loop3(next_loop_id++));
    }

    // prepare side verts
    std::vector<he::vert3*> old_vts, new_vts;
    old_vts.resize(num);
    new_vts.resize(num);
    auto old_e = old_loop.edge;
    auto new_e = new_loop.edge;
    for (size_t i = 0; i < num; ++i)
    {
        old_vts[i] = old_e->vert;
        new_vts[i] = new_e->vert;
        old_e = old_e->next;
        new_e = new_e->next;
    }

    // create side edges
    for (size_t i = 0; i < num; ++i)
    {
        he::edge3* edges[4];

        edges[0] = new he::edge3(old_vts[i],             side_faces[i], next_edge_id++);
        edges[1] = new he::edge3(old_vts[(i + 1) % num], side_faces[i], next_edge_id++);
        edges[2] = new he::edge3(new_vts[(i + 1) % num], side_faces[i], next_edge_id++);
        edges[3] = new he::edge3(new_vts[i],             side_faces[i], next_edge_id++);
        edges[0]->Connect(edges[1])->Connect(edges[2])->Connect(edges[3])->Connect(edges[0]);

        side_faces[i]->edge = edges[0];
    }

    // seam
    for (size_t i = 0; i < num; ++i)
    {
        auto& side0 = side_faces[i];
        auto& side1 = side_faces[(i + 1) % num];
        edge_make_pair(side0->edge->next, side1->edge->prev);
    }
}

}

namespace he
{

void Polyhedron::Fill()
{
    std::map<vert3*, edge3*> new_edges;

    // create edges
    auto first = m_edges.Head();
    auto curr = first;
    do {
        if (!curr->twin)
        {
            auto edge = new edge3(curr->next->vert, nullptr, m_next_edge_id++);
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
        edge3* edge = itr->second;
        if (edge->loop) {
            continue;
        }

        auto face = new loop3(m_next_loop_id++);
        m_faces.emplace_back(face);
        face->edge = edge;

        vert3* first = edge->vert;
        do {
            edge->loop = face;

            auto itr = new_edges.find(edge->twin->vert);
            assert(itr != new_edges.end());
            auto next_edge = itr->second;

            edge->Connect(next_edge);
            edge = next_edge;
        } while (edge->vert != first);

        m_loops.Append(face);
        m_faces.emplace_back(face);
    }
}

void Polyhedron::Fuse(float distance)
{
    std::vector<std::pair<he::vert3*, std::vector<he::edge3*>>> vert2edges;
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
            m_verts.Remove(itr1->first);
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
    std::vector<in_vert> verts;
    std::vector<in_face> faces;

    std::map<size_t, size_t> vert_uid2pos;
    for (auto& poly : polys)
    {
        auto first_vert = poly->GetVerts().Head();
        auto curr_vert = first_vert;
        do {
            auto ret = vert_uid2pos.insert({ curr_vert->ids.UID(), verts.size() });
            assert(ret.second);
            verts.push_back({ curr_vert->ids, curr_vert->position });
            curr_vert = curr_vert->linked_next;
        } while (curr_vert != first_vert);
    }

    for (auto& poly : polys)
    {
        auto first_l = poly->GetLoops().Head();
        auto curr_l = first_l;
        do {
            in_loop border;

            auto first_edge = curr_l->edge;
            auto curr_edge = first_edge;
            do {
                auto itr = vert_uid2pos.find(curr_edge->vert->ids.UID());
                assert(itr != vert_uid2pos.end());
                border.push_back(itr->second);

                curr_edge = curr_edge->next;
            } while (curr_edge != first_edge);

            std::vector<in_loop> holes;

            faces.emplace_back(curr_l->ids, border, holes);

            curr_l = curr_l->linked_next;
        } while (curr_l != first_l);
    }

    auto ret = std::make_shared<Polyhedron>(verts, faces);
    ret->Fuse(distance);
    return ret;
}

void Polyhedron::UniquePoints()
{
    Utility::UniquePoints(m_verts, m_edges, m_next_vert_id);
}

bool Polyhedron::Extrude(float distance, const std::vector<TopoID>& face_ids, bool create_face[ExtrudeMaxCount],
                         std::vector<Face>* new_faces)
{
    if (distance == 0) {
        return false;
    }

    const bool add_front = create_face[ExtrudeFront];
    const bool add_back  = create_face[ExtrudeBack];
    const bool add_side  = create_face[ExtrudeSide];

    std::vector<sm::Plane> planes;
    std::map<he::vert3*, std::vector<size_t>> vert2planes;
    planes.reserve(m_faces.size());
    for (auto& face : m_faces)
    {
        size_t idx = planes.size();

        sm::Plane plane;
        he::Utility::LoopToPlane(*face.border, plane);
        planes.push_back(plane);

        BuildMapVert2Planes(*face.border, idx, vert2planes);
        for (auto& hole : face.holes) {
            BuildMapVert2Planes(*hole, idx, vert2planes);
        }
    }

    std::vector<Face> old_front_faces;

    std::vector<vert3*> new_vts;
    std::map<vert3*, vert3*> vert_old2new;

    std::vector<Face> new_front_faces;

    size_t plane_idx = 0;
    for (auto& face : m_faces)
    {
        bool find = false;
        for (auto& id : face_ids) {
            if (id == face.border->ids) {
                find = true;
                break;
            }
        }

        if (find)
        {
            planes[plane_idx].dist -= distance;

            old_front_faces.push_back(face);

            Face new_face;
            new_face.border = CloneLoop(face.border, vert_old2new, new_vts,
                m_next_vert_id, m_next_edge_id, m_next_loop_id);
            new_face.holes.reserve(face.holes.size());
            for (auto& hole : face.holes) {
                new_face.holes.push_back(CloneLoop(hole, vert_old2new, new_vts,
                    m_next_vert_id, m_next_edge_id, m_next_loop_id));
            }
            new_front_faces.push_back(new_face);
        }

        ++plane_idx;
    }

    // offset verts
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
                sm::vec3 cross;
                bool intersect = sm::intersect_planes(p0, p1, p2, &cross);
                assert(intersect);
                itr.second->position = cross;
            }
            else
            {
                assert(p_ids.size() == 2);
                auto& p1 = planes[p_ids[1]];
                auto p2 = sm::Plane(p0.normal.Cross(edge->next->vert->position - edge->vert->position), itr.first->position);
                if (!sm::intersect_planes(p0, p1, p2, &itr.second->position))
                {
                    p2 = sm::Plane(p0.normal.Cross(prev_edge->next->vert->position - prev_edge->vert->position), itr.first->position);
                    sm::vec3 cross;
                    bool intersect = sm::intersect_planes(p0, p1, p2, &cross);
                    assert(intersect);
                    itr.second->position = cross;
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
            m_verts.Append(v);
        }
    } else {
        for (auto& v : new_vts) {
            delete v;
        }
    }
    // use new front faces
    if (add_front)
    {
        for (auto& f : new_front_faces)
        {
            m_loops.Append(f.border);
            for (auto& hole : f.holes) {
                m_loops.Append(hole);
            }
            m_faces.push_back(f);
        }
    }
    else
    {
        for (auto& f : new_front_faces) {
            delete f.border;
            for (auto& hole : f.holes) {
                delete hole;
            }
        }
    }
    // use new front edges
    if (add_front || add_side)
    {
        for (auto& f : new_front_faces) {
            AppendEdges(*f.border, m_edges);
            for (auto& hole : f.holes) {
                AppendEdges(*hole, m_edges);
            }
        }
    }
    else
    {
        for (auto& f : new_front_faces) {
            DeleteEdges(*f.border);
            for (auto& hole : f.holes) {
                DeleteEdges(*hole);
            }
        }
    }

    // create side faces
    std::vector<std::vector<std::vector<he::loop3*>>> side_faces;
    if (add_side)
    {
        assert(old_front_faces.size() == new_front_faces.size());
        for (size_t i = 0, n = old_front_faces.size(); i < n; ++i)
        {
            std::vector<std::vector<he::loop3*>> sides2;

            std::vector<he::loop3*> sides;
            CreateSideFaces(*old_front_faces[i].border, *new_front_faces[i].border, sides, m_next_loop_id, m_next_edge_id);
            sides2.push_back(sides);

            for (size_t j = 0, m = old_front_faces[i].holes.size(); j < m; ++j)
            {
                std::vector<he::loop3*> sides;
                CreateSideFaces(*old_front_faces[i].holes[j], *new_front_faces[i].holes[j], sides, m_next_loop_id, m_next_edge_id);
                sides2.push_back(sides);
            }

            side_faces.push_back(sides2);
        }

        for (auto& f3 : side_faces) {
            for (auto& f2 : f3) {
                for (auto& f : f2) {
                    AppendEdges(*f, m_edges);
                    m_loops.Append(f);
                    m_faces.emplace_back(f);
                }
            }
        }
    }


    // create back faces
    std::vector<Face> new_back_faces;
    std::vector<std::vector<edge3*>> new_back_edges;
    new_back_edges.resize(old_front_faces.size());
    if (add_back)
    {
        for (size_t i = 0, n = old_front_faces.size(); i < n; ++i)
        {
            auto& old_f = old_front_faces[i];

            Face new_f;

            auto new_border = new loop3(m_next_loop_id++);
            new_border->edge = Utility::CloneLoop(old_f.border, new_f.border, m_next_edge_id);
            Utility::FlipLoop(*new_border);
            new_f.border = new_border;
            m_loops.Append(new_border);
            AppendEdges(*new_border, m_edges);

            new_f.holes.reserve(old_f.holes.size());
            for (auto& hole : old_f.holes)
            {
                auto new_hole = new loop3(m_next_loop_id++);
                new_hole->edge = Utility::CloneLoop(hole, new_hole, m_next_edge_id);
                Utility::FlipLoop(*new_hole);
                new_f.holes.push_back(new_hole);
                m_loops.Append(new_hole);
                AppendEdges(*new_hole, m_edges);
            }

            new_back_faces.push_back(new_f);
            m_faces.push_back(new_f);
        }
    }

    // seam
    if (add_side && add_front)
    {
        assert(side_faces.size() == new_front_faces.size());
        for (size_t i = 0, n = side_faces.size(); i < n; ++i)
        {
            auto& front = new_front_faces[i];
            auto& sides = side_faces[i];
            assert(sides.size() == front.holes.size() + 1);

            size_t j = 0, k = 0;
            auto first_e = front.border->edge;
            auto curr_e = first_e;
            do {
                edge_make_pair(sides[j][k]->edge->next->next, curr_e);

                ++k;
                curr_e = curr_e->next;
            } while (curr_e != first_e);

            for (auto& hole : front.holes)
            {
                size_t k = 0;
                auto first_e = front.border->edge;
                auto curr_e = first_e;
                do {
                    edge_make_pair(sides[j][k]->edge->next->next, curr_e);

                    ++k;
                    curr_e = curr_e->next;
                } while (curr_e != first_e);
                ++j;
            }
        }
    }
    if (add_side && add_back)
    {
        assert(side_faces.size() == new_front_faces.size());
        for (size_t i = 0, n = side_faces.size(); i < n; ++i)
        {
            auto& front = new_front_faces[i];
            auto& sides = side_faces[i];
            assert(sides.size() == front.holes.size() + 1);

            size_t j = 0, k = 0;
            auto first_e = front.border->edge;
            auto curr_e = first_e;
            do {
                edge_make_pair(sides[j][k]->edge->next->next, curr_e->prev);

                ++k;
                curr_e = curr_e->next;
            } while (curr_e != first_e);

            for (auto& hole : front.holes)
            {
                size_t k = 0;
                auto first_e = front.border->edge;
                auto curr_e = first_e;
                do {
                    edge_make_pair(sides[j][k]->edge, curr_e->prev);

                    ++k;
                    curr_e = curr_e->next;
                } while (curr_e != first_e);
                ++j;
            }
        }
    }

    // rm old faces
    for (auto& f : old_front_faces) {
        RemoveFace(*this, f);
    }

    UpdateAABB();

    // return
    if (new_faces)
    {
        new_faces[ExtrudeFront] = new_front_faces;
        new_faces[ExtrudeBack]  = new_back_faces;

        new_faces[ExtrudeSide].clear();
        for (auto& f3 : side_faces) {
            for (auto& f2 : f3) {
                for (auto& f : f2) {
                    new_faces[ExtrudeSide].push_back(f);
                }
            }
        }
    }

    return true;
}

}