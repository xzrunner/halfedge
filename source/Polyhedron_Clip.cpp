#include "halfedge/Polyhedron.h"
#include "halfedge/Utility.h"

#include <set>
#include <map>

namespace
{

using PointStatus = he::Utility::PointStatus;

PointStatus CheckIntersects(const sm::Plane& plane, const he::DoublyLinkedList<he::vert3>& verts)
{
    size_t above = 0;
    size_t below = 0;
    size_t inside = 0;

    he::vert3* first = verts.Head();
    he::vert3* v = first;
    do {
        auto status = he::Utility::CalcPointPlaneStatus(plane, v->position);
        switch (status)
        {
        case PointStatus::Above:
            ++above;
            break;
        case PointStatus::Below:
            ++below;
            break;
        case PointStatus::Inside:
            ++inside;
            break;
        }
        v = v->linked_next;
    } while (v != first);

    const size_t sz = verts.Size();
    assert(above + below + inside == sz);

    if (above + inside == sz) {
        return PointStatus::Above;
    } else if (below + inside == sz) {
        return PointStatus::Below;
    } else {
        return PointStatus::Inside;
    }
}

he::edge3* FindInitialIntersectingEdge(const sm::Plane& plane, const he::DoublyLinkedList<he::edge3>& edges)
{
    auto first = edges.Head();
    auto curr = first;
    do {
        auto os = he::Utility::CalcPointPlaneStatus(plane, curr->vert->position);
        auto ds = he::Utility::CalcPointPlaneStatus(plane, curr->next->vert->position);

        if ((os == PointStatus::Inside && ds == PointStatus::Above) ||
            (os == PointStatus::Below  && ds == PointStatus::Above)) {
            if (curr->twin) {
                return curr->twin;
            }
        }
        if ((os == PointStatus::Above  && ds == PointStatus::Inside) ||
            (os == PointStatus::Above  && ds == PointStatus::Below)) {
            return curr;
        }

        if (os == PointStatus::Inside && ds == PointStatus::Inside)
        {
            auto next = curr->next;
            auto ss = he::Utility::CalcPointPlaneStatus(plane, next->next->vert->position);
            while (ss == PointStatus::Inside && next != curr) {
                next = next->next;
                ss = he::Utility::CalcPointPlaneStatus(plane, next->next->vert->position);
            }

            if (ss == PointStatus::Inside) {
                return nullptr;
            }

            if (ss == PointStatus::Below) {
                return curr->twin;
            } else {
                return curr;
            }
        }

        curr = curr->linked_next;
    } while (curr != first);

    return nullptr;
}

he::edge3* SplitEdgeByPlane(he::edge3* edge, const sm::Plane& plane,
                            he::DoublyLinkedList<he::vert3>& verts,
                            he::DoublyLinkedList<he::edge3>& edges,
                            size_t& next_vert_id, size_t& next_edge_id)
{
    auto& s_pos = edge->vert->position;
    auto& e_pos = edge->next->vert->position;

    auto s_dist = plane.GetDistance(s_pos);
    auto e_dist = plane.GetDistance(e_pos);

    assert(fabs(s_dist) > he::Utility::POINT_STATUS_EPSILON
        && fabs(e_dist) > he::Utility::POINT_STATUS_EPSILON
        && s_dist * e_dist < 0);

    float dot = s_dist / (s_dist - e_dist);
    assert(dot > 0.0f && dot < 1.0f);

    auto pos = s_pos + (e_pos - s_pos) * dot;
    auto new_vert = new he::vert3(pos, next_vert_id++);
    verts.Append(new_vert);
    auto new_edge = new he::edge3(new_vert, edge->loop, edge->ids);
    new_edge->ids.Append(next_edge_id++);
    edges.Append(new_edge);

    edge->ids.Append(next_edge_id++);
    auto edge_next = edge->next;
    edge->Connect(new_edge)->Connect(edge_next);

    auto twin_edge = edge->twin;
    if (twin_edge)
    {
        auto new_twin_edge = new he::edge3(new_vert, twin_edge->loop, twin_edge->ids);
        new_twin_edge->ids.Append(next_edge_id++);
        edges.Append(new_twin_edge);

        twin_edge->ids.Append(next_edge_id++);
        auto twin_edge_next = twin_edge->next;
        twin_edge->Connect(new_twin_edge)->Connect(twin_edge_next);

        edge_make_pair(new_edge, twin_edge);
        edge_make_pair(new_twin_edge, edge);
    }

    return new_edge;
}

void IntersectWithPlane(he::edge3* old_boundary_first,
                        he::edge3* new_boundary_first,
                        he::DoublyLinkedList<he::edge3>& edges,
                        he::DoublyLinkedList<he::loop3>& loops,
                        size_t& next_edge_id, size_t& next_loop_id,
                        std::vector<he::Polyhedron::Face>& faces)
{
    he::edge3* new_boundary_last = old_boundary_first->prev;

    auto old_loop = old_boundary_first->loop;
    he::edge3* old_boundary_splitter = new he::edge3(new_boundary_first->vert, old_loop, next_edge_id++);
    he::edge3* new_boundary_splitter = new he::edge3(old_boundary_first->vert, old_loop, next_edge_id++);
    he::edge_make_pair(old_boundary_splitter, new_boundary_splitter);

    auto new_boundary_first_prev = new_boundary_first->prev;
    old_boundary_first->prev->Connect(new_boundary_splitter);
    new_boundary_splitter->Connect(new_boundary_first);
    new_boundary_first_prev->Connect(old_boundary_splitter);
    old_boundary_splitter->Connect(old_boundary_first);

    auto new_loop = new he::loop3(new_boundary_first->loop->ids);
    new_loop->ids.Append(next_loop_id++);
    he::bind_edge_loop(new_loop, new_boundary_first);

    old_loop->ids.Append(next_loop_id++);
    he::bind_edge_loop(old_loop, old_boundary_first);

    edges.Append(old_boundary_splitter);
    edges.Append(new_boundary_splitter);
    loops.Append(new_loop);

    faces.emplace_back(new_loop);
}

he::edge3* IntersectWithPlane(he::edge3* first_boundary_edge, const sm::Plane& plane,
                              he::DoublyLinkedList<he::vert3>& verts,
                              he::DoublyLinkedList<he::edge3>& edges,
                              he::DoublyLinkedList<he::loop3>& loops,
                              size_t& next_vert_id, size_t& next_edge_id, size_t& next_loop_id,
                              std::vector<he::Polyhedron::Face>& faces)
{
    he::edge3* seam_ori = nullptr;
    he::edge3* seam_dst = nullptr;
    he::edge3* seam_dst_prev = nullptr;
    he::edge3* curr_boundary_edge_prev = nullptr;

    he::edge3* curr_boundary_edge = first_boundary_edge;
    do {
        PointStatus os = he::Utility::CalcPointPlaneStatus(plane, curr_boundary_edge->vert->position);
        PointStatus ds = he::Utility::CalcPointPlaneStatus(plane, curr_boundary_edge->next->vert->position);

        if (os == PointStatus::Inside)
        {
            if (seam_ori == nullptr) {
                seam_ori = curr_boundary_edge;
            } else {
                seam_dst = curr_boundary_edge;
            }
            curr_boundary_edge = curr_boundary_edge->next;
        }
        else if ((os == PointStatus::Below && ds == PointStatus::Above) ||
                 (os == PointStatus::Above && ds == PointStatus::Below))
        {
            SplitEdgeByPlane(curr_boundary_edge, plane, verts, edges, next_vert_id, next_edge_id);
            curr_boundary_edge = curr_boundary_edge->next;

            auto new_vertex = curr_boundary_edge->vert;
            assert(he::Utility::CalcPointPlaneStatus(plane, new_vertex->position) == PointStatus::Inside);
        }
        else
        {
            curr_boundary_edge = curr_boundary_edge->next;
        }
    } while (seam_dst == nullptr && curr_boundary_edge != first_boundary_edge);
    assert(seam_ori != nullptr);

    if (seam_dst == nullptr) {
        return seam_ori->prev;
    }

    if (seam_dst->next == seam_ori)
    {
        std::swap(seam_ori, seam_dst);
    }
    else if (seam_ori->next != seam_dst)
    {
        auto os = he::Utility::CalcPointPlaneStatus(plane, seam_ori->next->vert->position);
        assert(os != PointStatus::Inside);
        if (os == PointStatus::Below) {
            IntersectWithPlane(seam_ori, seam_dst, edges, loops, next_edge_id, next_loop_id, faces);
        } else {
            IntersectWithPlane(seam_dst, seam_ori, edges, loops, next_edge_id, next_loop_id, faces);
        }
    }

    return seam_dst->prev;
}

he::edge3* FindNextIntersectingEdge(he::edge3* search_from, const sm::Plane& plane)
{
    auto curr_edge = search_from->next;
    auto stop_edge = search_from->twin;
    do {
        assert(curr_edge != stop_edge);

        auto cd = curr_edge->next->vert;
        auto po = curr_edge->prev->vert;
        auto cds = he::Utility::CalcPointPlaneStatus(plane, cd->position);
        auto pos = he::Utility::CalcPointPlaneStatus(plane, po->position);
        if ((cds == PointStatus::Inside) ||
            (cds == PointStatus::Below && pos == PointStatus::Above) ||
            (cds == PointStatus::Above && pos == PointStatus::Below)) {
            return curr_edge;
        }

        if (!curr_edge->twin) {
            break;
        }
        curr_edge = curr_edge->twin->next;
    } while (curr_edge != stop_edge);

    return nullptr;
}

std::vector<he::edge3*> IntersectWithPlaneImpl(he::edge3* start_edge, const sm::Plane& plane,
                                               he::DoublyLinkedList<he::vert3>& verts,
                                               he::DoublyLinkedList<he::edge3>& edges,
                                               he::DoublyLinkedList<he::loop3>& loops,
                                               size_t& next_vert_id, size_t& next_edge_id, size_t& next_loop_id,
                                               std::vector<he::Polyhedron::Face>& faces)
{
    std::vector<he::edge3*> seam;

    auto curr_edge = start_edge;
    auto stop_vert = curr_edge->next->vert;
    do {
        curr_edge = FindNextIntersectingEdge(curr_edge, plane);
        if (!curr_edge) {
            return std::vector<he::edge3*>();
        }

        curr_edge = IntersectWithPlane(curr_edge, plane, verts, edges, loops,
            next_vert_id, next_edge_id, next_loop_id, faces);
        seam.push_back(curr_edge);
    } while (curr_edge->next->vert != stop_vert);

    if (seam.empty()) {
        seam.push_back(start_edge);
    }

    return seam;
}

std::vector<he::edge3*> IntersectWithPlane(const sm::Plane& plane,
                                           he::DoublyLinkedList<he::vert3>& verts,
                                           he::DoublyLinkedList<he::edge3>& edges,
                                           he::DoublyLinkedList<he::loop3>& loops,
                                           size_t& next_vert_id, size_t& next_edge_id, size_t& next_loop_id,
                                           std::vector<he::Polyhedron::Face>& faces)
{
    std::vector<he::edge3*> seam;

    auto init_edge = FindInitialIntersectingEdge(plane, edges);
    assert(init_edge);

    auto start_edge = IntersectWithPlane(init_edge, plane, verts, edges, loops, next_vert_id, next_edge_id, next_loop_id, faces);
    seam = IntersectWithPlaneImpl(start_edge, plane, verts, edges, loops, next_vert_id, next_edge_id, next_loop_id, faces);
    if (seam.empty()) {
        seam = IntersectWithPlaneImpl(start_edge->twin, plane, verts, edges, loops, next_vert_id, next_edge_id, next_loop_id, faces);
    }

    if (seam.empty()) {
        seam.push_back(start_edge);
    }

    return seam;
}

bool HasMultipleLoops(const std::vector<he::edge3*>& seam)
{
    std::set<he::vert3*> visited;
    for (auto& e : seam) {
        if (!visited.insert(e->vert).second) {
            return true;
        }
    }
    return false;
}

void OnVertexInvalid(he::DoublyLinkedList<he::vert3>& verts,
    he::DoublyLinkedList<he::edge3>& edges, he::DoublyLinkedList<he::loop3>& loops);

void OnEdgeInvalid(he::DoublyLinkedList<he::vert3>& verts,
                   he::DoublyLinkedList<he::edge3>& edges,
                   he::DoublyLinkedList<he::loop3>& loops)
{
    bool edge_dirty = false;

    auto curr_l = loops.Head();
    auto first_l = curr_l;
    do {
        if (curr_l->ids.IsValid())
        {
            auto curr_edge = curr_l->edge;
            auto first_edge = curr_edge;
            do {
                if (!curr_edge->ids.IsValid()) {
                    curr_l->ids.MakeInvalid();
                    break;
                }
                curr_edge = curr_edge->next;
            } while (curr_edge != first_edge);

            if (!curr_l->ids.IsValid())
            {
                auto curr_edge = curr_l->edge;
                auto first_edge = curr_edge;
                do {
                    if (curr_edge->ids.IsValid()) {
                        curr_edge->ids.MakeInvalid();
                        edge_dirty = true;
                    }
                    curr_edge = curr_edge->next;
                } while (curr_edge != first_edge);
            }
        }

        curr_l = curr_l->linked_next;
    } while (curr_l != first_l);

    if (edge_dirty) {
        OnEdgeInvalid(verts, edges, loops);
    }
}

void OnVertexInvalid(he::DoublyLinkedList<he::vert3>& verts,
                     he::DoublyLinkedList<he::edge3>& edges,
                     he::DoublyLinkedList<he::loop3>& loops)
{
    bool edge_dirty = false;

    auto curr_edge = edges.Head();
    auto first_edge = curr_edge;
    do {
        if (curr_edge->ids.IsValid() && !curr_edge->vert->ids.IsValid()) {
            curr_edge->ids.MakeInvalid();
            edge_dirty = true;
        }

        curr_edge = curr_edge->linked_next;
    } while (curr_edge != first_edge);

    if (edge_dirty) {
        OnEdgeInvalid(verts, edges, loops);
    }
}

bool FixVertexInvalid(he::DoublyLinkedList<he::vert3>& verts,
                      he::DoublyLinkedList<he::edge3>& edges)
{
    bool vert_dirty = false;

    // map vertex to edges
    std::map<he::vert3*, std::vector<he::edge3*>> vert2edges;
    auto curr_edge = edges.Head();
    auto first_edge = curr_edge;
    do {
        auto itr = vert2edges.find(curr_edge->vert);
        if (itr != vert2edges.end()) {
            itr->second.push_back(curr_edge);
        } else {
            vert2edges.insert({ curr_edge->vert, { curr_edge } });
        }

        curr_edge = curr_edge->linked_next;
    } while (curr_edge != first_edge);

    // fix verts
    auto first_vert = verts.Head();
    auto curr_vert = first_vert;
    do {
        if (!curr_vert->ids.IsValid() ||
            curr_vert->ids.IsValid() && curr_vert->edge->ids.IsValid()) {
            curr_vert = curr_vert->linked_next;
            continue;
        }

        auto itr = vert2edges.find(curr_vert);
        assert(itr != vert2edges.end());
        bool find = false;
        for (auto& e : itr->second) {
            if (e->ids.IsValid()) {
                curr_vert->edge = e;
                find = true;
                break;
            }
        }
        if (!find) {
            curr_vert->ids.MakeInvalid();
            vert_dirty = true;
        }

        curr_vert = curr_vert->linked_next;
    } while (curr_vert != first_vert);

    return vert_dirty;
}

void FixEdgeInvalid(he::DoublyLinkedList<he::edge3>& edges)
{
    auto first_edge = edges.Head();
    auto curr_edge = first_edge;
    do {
        if (curr_edge->ids.IsValid())
        {
            assert(curr_edge->vert->ids.IsValid());
            assert(curr_edge->loop->ids.IsValid());
            assert(curr_edge->prev->ids.IsValid());
            assert(curr_edge->next->ids.IsValid());
            if (curr_edge->twin && !curr_edge->twin->ids.IsValid()) {
                curr_edge->twin = nullptr;
            }
        }

        curr_edge = curr_edge->linked_next;
    } while (curr_edge != first_edge);
}

void FixLoopInvalid(he::DoublyLinkedList<he::loop3>& loops)
{
    auto first_l = loops.Head();
    auto curr_l = first_l;
    do {
        if (curr_l->edge->ids.IsValid())
        {
            auto first_edge = curr_l->edge;
            auto curr_edge = first_edge;
            do {
                if (curr_edge->ids.IsValid()) {
                    curr_l->edge = curr_edge;
                    break;
                }

                curr_edge = curr_edge->next;
            } while (curr_edge != first_edge);

            assert(curr_l->edge->ids.IsValid());
        }

        curr_l = curr_l->linked_next;
    } while (curr_l != first_l);
}


void DeleteInvalid(std::vector<he::Polyhedron::Face>& faces)
{
    for (auto itr = faces.begin(); itr != faces.end(); )
    {
        bool valid = true;
        if (!itr->border->ids.IsValid()) {
            valid = false;
        }
        for (auto& hole : itr->holes) {
            if (!itr->border->ids.IsValid()) {
                valid = false;
                break;
            }
        }

        if (!valid) {
            itr = faces.erase(itr);
        } else {
            itr++;
        }
    }
}

template <typename T>
void DeleteInvalid(he::DoublyLinkedList<T>& list)
{
    std::vector<T*> invalid;
    auto first = list.Head();
    auto curr = first;
    do {
        if (!curr->ids.IsValid()) {
            invalid.push_back(curr);
        }
        curr = curr->linked_next;
    } while (curr != first);

    for (auto& i : invalid) {
        list.Remove(i);
    }
    for (auto& i : invalid) {
        delete i;
    }
}

void DeleteByPlane(const sm::Plane& plane, bool del_above,
                   he::DoublyLinkedList<he::vert3>& verts,
                   he::DoublyLinkedList<he::edge3>& edges,
                   he::DoublyLinkedList<he::loop3>& loops,
                   std::vector<he::Polyhedron::Face>& faces)
{
    bool vert_dirty = false;

    auto curr_vert = verts.Head();
    auto first_vert = curr_vert;
    do {
        auto st = he::Utility::CalcPointPlaneStatus(plane, curr_vert->position);
        if ((st == PointStatus::Above && del_above) ||
            (st == PointStatus::Below && !del_above)) {
            curr_vert->ids.MakeInvalid();
            vert_dirty = true;
        }

        curr_vert = curr_vert->linked_next;
    } while (curr_vert != first_vert);

    if (!vert_dirty) {
        return;
    }

    do {
        OnVertexInvalid(verts, edges, loops);
    } while (FixVertexInvalid(verts, edges));
    FixEdgeInvalid(edges);
    FixLoopInvalid(loops);

    DeleteInvalid(faces);

    DeleteInvalid(verts);
    DeleteInvalid(edges);
    DeleteInvalid(loops);
}

sm::cube CalcAABB(const he::DoublyLinkedList<he::vert3>& verts)
{
    if (verts.Size() == 0) {
        return sm::cube();
    }

    sm::cube aabb;
    auto first_vertex = verts.Head();
    auto curr_vertex = first_vertex;
    do {
        aabb.Combine(curr_vertex->position);
        curr_vertex = curr_vertex->linked_next;
    } while (curr_vertex != first_vertex);
    return aabb;
}

}

namespace he
{

bool Polyhedron::Clip(const sm::Plane& plane, KeepType keep, bool seam_face)
{
    auto st = CheckIntersects(plane, m_verts);
    switch (st)
    {
    case PointStatus::Above:
        return keep == KeepType::KeepAll || keep == KeepType::KeepAbove;
    case PointStatus::Below:
        return keep == KeepType::KeepAll || keep == KeepType::KeepBelow;
    case PointStatus::Inside:
        break;
    default:
        assert(0);
    }

    auto seam = IntersectWithPlane(plane, m_verts, m_edges, m_loops,
        m_next_vert_id, m_next_edge_id, m_next_loop_id, m_faces);
    if (seam.empty()) {
        return false;
    }

    if (keep == KeepType::KeepAll) {
        return true;
    }

    if (seam_face && keep == KeepType::KeepBelow)
    {
        for (auto& s : seam) {
            s = s->twin;
        }
        std::reverse(seam.begin(), seam.end());
    }

    if (seam_face)
    {
//        assert(!seam.front()->twin);

        auto new_loop = new loop3(m_next_loop_id++);
        m_loops.Append(new_loop);
        m_faces.emplace_back(new_loop);

        std::vector<edge3*> new_edges;
        new_edges.reserve(seam.size());
        for (int i = 0, n = seam.size(); i < n; ++i)
        {
            edge3* new_edge = new edge3(seam[(i + 1) % n]->vert, new_loop, m_next_edge_id++);
            edge_make_pair(new_edge, seam[i]);
            new_edges.push_back(new_edge);
            m_edges.Append(new_edge);
        }

        for (auto& e : seam) {
            assert(e->next->vert == e->twin->vert);
        }

        if (new_edges.size() > 1) {
            for (int i = 0, n = new_edges.size(); i < n; ++i) {
                new_edges[i]->Connect(new_edges[(i - 1 + n) % n]);
            }
        }

        for (auto& e : seam) {
            assert(e->next->vert == e->twin->vert);
        }

        new_loop->edge = new_edges[0];
    }

    DeleteByPlane(plane, keep == KeepType::KeepBelow, m_verts, m_edges, m_loops, m_faces);

    m_aabb = CalcAABB(m_verts);

    return true;
}

}