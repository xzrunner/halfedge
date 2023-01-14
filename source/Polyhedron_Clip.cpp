#include "halfedge/Polyhedron.h"
#include "halfedge/Utility.h"

#include <SM_Calc.h>

#include <set>
#include <map>
#include <iterator>

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

// bool: intersected
std::pair<bool, he::edge3*> FindInitialIntersectingEdge(const sm::Plane& plane, const he::DoublyLinkedList<he::edge3>& edges)
{
    auto first = edges.Head();
    auto curr = first;
    do {
        auto os = he::Utility::CalcPointPlaneStatus(plane, curr->vert->position);
        auto ds = he::Utility::CalcPointPlaneStatus(plane, curr->next->vert->position);

        if ((os == PointStatus::Inside && ds == PointStatus::Above) ||
            (os == PointStatus::Below  && ds == PointStatus::Above)) {
            if (curr->twin) {
                return { true, curr->twin };
            }
        }
        if ((os == PointStatus::Above  && ds == PointStatus::Inside) ||
            (os == PointStatus::Above  && ds == PointStatus::Below)) {
            return { true, curr };
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
                return { false, curr };
            }

            if (ss == PointStatus::Below) {
                return { true, curr->twin };
            } else {
                return { true, curr };
            }
        }

        curr = curr->linked_next;
    } while (curr != first);

    return { true, nullptr };
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

        he::edge_del_pair(edge);
        he::edge_del_pair(twin_edge);
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
    auto test_edge = [](he::edge3* curr_edge, const sm::Plane& plane) -> bool
    {
        auto cd = curr_edge->next->vert;
        auto po = curr_edge->prev->vert;

        auto cds = he::Utility::CalcPointPlaneStatus(plane, cd->position);
        auto pos = he::Utility::CalcPointPlaneStatus(plane, po->position);
        if ((cds == PointStatus::Inside) ||
            (cds == PointStatus::Below && pos == PointStatus::Above) ||
            (cds == PointStatus::Above && pos == PointStatus::Below)) {
            return true;
        } else {
            return false;
        }
    };

    auto curr_edge = search_from->next;
    auto stop_edge = search_from->twin;
    do {
        assert(curr_edge != stop_edge);

        if (test_edge(curr_edge, plane)) {
            return curr_edge;
        }

        if (!curr_edge->twin) {
            break;
        }
        curr_edge = curr_edge->twin->next;
    } while (curr_edge != stop_edge);

    if (test_edge(stop_edge, plane)) {
        return stop_edge;
    }

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

    auto find = FindInitialIntersectingEdge(plane, edges);
    if (find.first) 
    {
        assert(find.second);

        auto init_edge = find.second;

        auto start_edge = IntersectWithPlane(init_edge, plane, verts, edges, loops, next_vert_id, next_edge_id, next_loop_id, faces);
        seam = IntersectWithPlaneImpl(start_edge, plane, verts, edges, loops, next_vert_id, next_edge_id, next_loop_id, faces);
        if (seam.empty() && start_edge->twin) {
            seam = IntersectWithPlaneImpl(start_edge->twin, plane, verts, edges, loops, next_vert_id, next_edge_id, next_loop_id, faces);
        }
    }
    else
    {
        auto first = find.second;
        seam.push_back(first);
        auto next = first->next;
        while (next != first) {
            seam.push_back(next);
            next = next->next;
        }
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
                edge_del_pair(curr_edge);
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

bool IsLoopValid(const he::loop3* loop)
{
    if (!loop->ids.IsValid()) {
        return false;
    }

    int num = 0;

    auto first_e = loop->edge;
    auto curr_e = first_e;
    do {
        ++num;
        curr_e = curr_e->next;
    } while (curr_e && curr_e != first_e);

    if (num < 3) {
        return false;
    }

    return true;
}

void DeleteInvalid(std::vector<he::Polyhedron::Face>& faces)
{
    for (auto itr = faces.begin(); itr != faces.end(); )
    {
        bool valid = true;
        if (!IsLoopValid(itr->border)) {
            valid = false;
        }
        for (auto& hole : itr->holes) {
            if (!IsLoopValid(hole)) {
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

    DeleteInvalid(faces);
}

int GetNoTwinEdgesNum(const he::DoublyLinkedList<he::edge3>& edges)
{
    int ret = 0;

    auto first = edges.Head();
    auto curr = first;
    do {
        if (!curr->twin || !curr->twin->ids.IsValid()) {
            ++ret;
        }
        curr = curr->linked_next;
    } while (curr != first);

    return ret;
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

void fix_seam_order(std::vector<he::edge3*>& seam, const sm::Plane& plane, he::Polyhedron::KeepType keep)
{
    std::vector<sm::vec3> loop;
    loop.reserve(seam.size());
    for (auto& e : seam) {
        loop.push_back(e->vert->position);
    }

    auto need_dir = plane.normal;
    if (keep == he::Polyhedron::KeepType::KeepAbove) {
        need_dir = -need_dir;
    }

    auto loop_dir = sm::calc_face_normal(loop);
    if (need_dir.Dot(loop_dir) > 0)
    {
        for (auto& s : seam) {
            if (!s->twin) {
                return;
            }

            s = s->twin;
        }
        std::reverse(seam.begin(), seam.end());
    }
}

void rm_face(he::loop3* loop, std::vector<he::Polyhedron::Face>& faces)
{
    for (auto itr = faces.begin(); itr != faces.end(); ++itr)
    {
        if (itr->border == loop) {
            faces.erase(itr);
            break;
        }
    }
}

void sew_seam(const he::loop3* cover, const he::loop3* cover2,  
              he::DoublyLinkedList<he::loop3>& loops,
              he::DoublyLinkedList<he::edge3>& edges,
              he::DoublyLinkedList<he::vert3>& verts,
              std::vector<he::Polyhedron::Face>& faces)
{
    std::vector<he::edge3*> edges0, edges1;

    auto curr_edge = cover->edge;
    auto first_edge = curr_edge;
    do {
        edges0.push_back(curr_edge);
        curr_edge = curr_edge->next;
    } while (curr_edge != first_edge);

    curr_edge = cover2->edge;
    first_edge = curr_edge;
    do {
        edges1.push_back(curr_edge);
        curr_edge = curr_edge->next;
    } while (curr_edge != first_edge);

    std::vector<he::edge3*> del_edges;

    assert(edges0.size() == edges1.size());
    for (int i = 0, n = edges0.size(); i < n; ++i)
    {
        // n1    p1
        // ^     |
        // |     |
        // |     V
        // <-----o
        // 
        // o----->
        // ^     |
        // |     |
        // |     V
        // p0    n0

        auto p0 = edges0[i]->twin->prev->vert->position;
        auto n0 = edges0[i]->twin->next->vert->position;
        auto p1 = edges1[i]->twin->prev->vert->position;
        auto n1 = edges1[i]->twin->next->vert->position;

        assert(p0.x == n1.x && p0.z == n1.z
            && p1.x == n0.x && p1.z == n0.z);

        edges0[i]->twin->next->vert->position = edges1[i]->twin->prev->vert->position;
        assert(edges1[i]->twin->prev->prev->twin->vert == edges1[i]->twin->prev->vert);
        edges1[i]->twin->prev->prev->twin->vert = edges0[i]->twin->next->vert;
        assert(edges1[i]->twin->prev->twin->next->vert == edges1[i]->twin->prev->vert);
        edges1[i]->twin->prev->twin->next->vert = edges0[i]->twin->next->vert;

        del_edges.push_back(edges1[i]->twin->prev);
        del_edges.push_back(edges1[i]->twin->next);

        edges0[i]->twin->prev->Connect(edges1[i]->twin->next->next);
        edges1[i]->twin->prev->prev->Connect(edges0[i]->twin->next);

        verts.Remove(edges1[i]->twin->prev->vert);
        verts.Remove(edges1[i]->twin->next->vert);

        rm_face(edges1[i]->twin->loop, faces);

        loops.Remove(edges1[i]->twin->loop);
        delete edges1[i]->twin->loop;
    }

    for (auto e : del_edges) {
        edges.Remove(e);
        delete e;
    }
}

void rm_loop(he::loop3* loop, he::DoublyLinkedList<he::loop3>& loops, 
             he::DoublyLinkedList<he::edge3>& edges,
             std::vector<he::Polyhedron::Face>& faces)
{
    rm_face(loop, faces);

    auto curr_edge = loop->edge;
    auto first_edge = curr_edge;
    do {
        assert(curr_edge->twin->twin == curr_edge);
        curr_edge->twin->twin = nullptr;

        edges.Remove(curr_edge->twin);
        delete curr_edge->twin;

        edges.Remove(curr_edge);
        auto next_edge = curr_edge->next;
        delete curr_edge;

        curr_edge = next_edge;
    } while (curr_edge != first_edge);

    loops.Remove(loop);
    delete loop;
}

void separate(he::Polyhedron* poly, const sm::Plane& plane, 
              he::DoublyLinkedList<he::vert3>& new_verts,
              he::DoublyLinkedList<he::edge3>& new_edges,
              he::DoublyLinkedList<he::loop3>& new_loops,
              std::vector<he::Polyhedron::Face>& new_faces)
{
    std::set<he::vert3*> verts_up;
    he::edge3* first = poly->GetEdges().Head();
    he::edge3* e = first;
    do {
        auto os = he::Utility::CalcPointPlaneStatus(plane, e->vert->position);
        switch (os)
        {
        case PointStatus::Above:
            verts_up.insert(e->vert);
            break;
        case PointStatus::Inside:
            if (he::Utility::CalcPointPlaneStatus(plane, e->next->vert->position) == PointStatus::Above) {
                verts_up.insert(e->vert);
            }
            break;
        }

        e = e->linked_next;
    } while (e != first);

    auto& ori_verts = const_cast<he::DoublyLinkedList<he::vert3>&>(poly->GetVerts());
    he::vert3* first_v = ori_verts.Head();
    he::vert3* v = first_v;
    do {
        if (verts_up.find(v) != verts_up.end()) {
            v = v->linked_next;
        } else {
            auto next = ori_verts.Remove(v);
            new_verts.Append(v);
            v = next;
        }
    } while (v != first_v);

    auto& ori_edges = const_cast<he::DoublyLinkedList<he::edge3>&>(poly->GetEdges());
    he::edge3* first_e = ori_edges.Head();
    e = first_e;
    do {
        if (verts_up.find(e->vert) != verts_up.end()) {
            e = e->linked_next;
        } else {
            auto next = ori_edges.Remove(e);
            new_edges.Append(e);
            e = next;
        }
    } while (e != first_e);

    auto& ori_loops = const_cast<he::DoublyLinkedList<he::loop3>&>(poly->GetLoops());
    he::loop3* first_l = ori_loops.Head();
    auto l = first_l;
    do {
        if (verts_up.find(l->edge->vert) != verts_up.end()) {
            l = l->linked_next;
        } else {
            auto next = ori_loops.Remove(l);
            new_loops.Append(l);
            l = next;
        }
    } while (l != first_l);

    auto& ori_faces = const_cast<std::vector<he::Polyhedron::Face>&>(poly->GetFaces());
    for (auto itr = ori_faces.begin(); itr != ori_faces.end(); )
    {
        if (verts_up.find(itr->border->edge->vert) != verts_up.end()) {
            ++itr;
        } else {
            new_faces.push_back(*itr);
            itr = ori_faces.erase(itr);
        }
    }
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

    if (seam_face)
    {
        fix_seam_order(seam, plane, keep);

//        assert(!seam.front()->twin);

        auto new_loop = new loop3(m_next_loop_id++);
        m_loops.Append(new_loop);
        m_faces.emplace_back(new_loop);

        std::vector<edge3*> new_edges;
        new_edges.reserve(seam.size());
        for (int i = 0, n = seam.size(); i < n; ++i)
        {
            edge3* new_edge = new edge3(seam[(i + 1) % n]->vert, new_loop, m_next_edge_id++);

            edge_del_pair(seam[i]);
            edge_make_pair(new_edge, seam[i]);
            new_edges.push_back(new_edge);
            m_edges.Append(new_edge);
        }

        if (new_edges.size() > 1) {
            for (int i = 0, n = new_edges.size(); i < n; ++i) {
                new_edges[i]->Connect(new_edges[(i - 1 + n) % n]);
            }
        }

        new_loop->edge = new_edges[0];
    }

    if (keep != KeepType::KeepAll) {
        DeleteByPlane(plane, keep == KeepType::KeepBelow, m_verts, m_edges, m_loops, m_faces);
    }

    assert(GetNoTwinEdgesNum(m_edges) == 0);

    m_aabb = CalcAABB(m_verts);

    return true;
}

std::shared_ptr<Polyhedron> Polyhedron::Fork(const sm::Plane& plane)
{
    auto seam = IntersectWithPlane(plane, m_verts, m_edges, m_loops,
        m_next_vert_id, m_next_edge_id, m_next_loop_id, m_faces);
    if (seam.empty()) {
        return false;
    }

    // clone middle pos
    for (auto edge : seam)
    {
        vert3* new_vert = new vert3(edge->vert->position, m_next_vert_id++);
        edge->vert = new_vert;
        edge->prev->twin->vert = new_vert;
        m_verts.Append(new_vert);
    }

    auto seam2face = [&](const std::vector<edge3*>& edges) -> loop3*
    {
        auto new_loop = new loop3(m_next_loop_id++);
        m_loops.Append(new_loop);
        m_faces.emplace_back(new_loop);

        std::vector<edge3*> new_edges;
        new_edges.reserve(edges.size());
        for (int i = 0, n = edges.size(); i < n; ++i)
        {
            edge3* new_edge = new edge3(edges[(i + 1) % n]->vert, new_loop, m_next_edge_id++);

            edge_del_pair(edges[i]);
            edge_make_pair(new_edge, edges[i]);
            new_edges.push_back(new_edge);
            m_edges.Append(new_edge);
        }

        if (new_edges.size() > 1) {
            for (int i = 0, n = new_edges.size(); i < n; ++i) {
                new_edges[i]->Connect(new_edges[(i - 1 + n) % n]);
            }
        }

        new_loop->edge = new_edges[0];

        return new_loop;
    };

    std::vector<edge3*> twin_seam;
    for (auto& edge : seam) {
        twin_seam.push_back(edge->twin);
    }

    auto cover = seam2face(seam);
    auto cover2 = seam2face(twin_seam);
    //out_seam.push_back(cover);
    //out_seam.push_back(cover2);

    auto ret = std::make_shared<Polyhedron>();
    separate(this, plane, ret->m_verts, ret->m_edges, ret->m_loops, ret->m_faces);
    UpdateAABB();
    ret->UpdateAABB();

    return ret;
}

bool Polyhedron::Join(const std::shared_ptr<Polyhedron>& poly)
{
    // todo: check seams valid
    auto seam0 = m_loops.Head()->linked_prev;
    auto seam1 = poly->GetLoops().Head()->linked_prev;

    m_verts.Connect(poly->m_verts);
    m_edges.Connect(poly->m_edges);
    m_loops.Connect(poly->m_loops);

    std::copy(poly->m_faces.begin(), poly->m_faces.end(), std::back_inserter(m_faces));
    poly->m_faces.clear();

    sew_seam(seam0, seam1, m_loops, m_edges, m_verts, m_faces);
    rm_loop(seam0, m_loops, m_edges, m_faces);
    rm_loop(seam1, m_loops, m_edges, m_faces);

    //// offset
    //std::set<he::vert3*> verts_up;
    //he::edge3* first = m_edges.Head();
    //he::edge3* e = first;
    //do {
    //    if (e->vert->position.y > 0) {
    //        verts_up.insert(e->vert);
    //    } else if (e->vert->position.y == 0 && e->next && e->next->vert->position.y > 0) {
    //        verts_up.insert(e->vert);
    //    }
    //    e = e->linked_next;
    //} while (e != first);
    //for (auto v : verts_up) {
    //    v->position.x += 2.0f;
    //}

    return true;
}

}