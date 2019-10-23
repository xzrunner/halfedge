#include "halfedge/Polyhedron.h"

#include <set>
#include <map>

namespace
{

const float POINT_STATUS_EPSILON = 0.0001f;

enum class PointStatus
{
    Above,
    Below,
    Inside,
};

PointStatus CalcPointStatus(const sm::Plane& plane, const sm::vec3& pos)
{
    const float dist = plane.GetDistance(pos);
    if (dist > POINT_STATUS_EPSILON) {
        return PointStatus::Above;
    } else if (dist < -POINT_STATUS_EPSILON) {
        return PointStatus::Below;
    } else {
        return PointStatus::Inside;
    }
}

PointStatus CheckIntersects(const sm::Plane& plane, const he::DoublyLinkedList<he::Vertex>& vertices)
{
    size_t above = 0;
    size_t below = 0;
    size_t inside = 0;

    he::Vertex* first = vertices.Head();
    he::Vertex* v = first;
    do {
        auto status = CalcPointStatus(plane, v->position);
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

    const size_t sz = vertices.Size();
    assert(above + below + inside == sz);

    if (above + inside == sz) {
        return PointStatus::Above;
    } else if (below + inside == sz) {
        return PointStatus::Below;
    } else {
        return PointStatus::Inside;
    }
}

he::Edge* FindInitialIntersectingEdge(const sm::Plane& plane, const he::DoublyLinkedList<he::Edge>& edges)
{
    auto first = edges.Head();
    auto curr = first;
    do {
        auto os = CalcPointStatus(plane, curr->vert->position);
        auto ds = CalcPointStatus(plane, curr->next->vert->position);

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
            auto ss = CalcPointStatus(plane, next->next->vert->position);
            while (ss == PointStatus::Inside && next != curr) {
                next = next->next;
                ss = CalcPointStatus(plane, next->next->vert->position);
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

he::Edge* SplitEdgeByPlane(he::Edge* edge, const sm::Plane& plane,
                           he::DoublyLinkedList<he::Vertex>& vertices,
                           he::DoublyLinkedList<he::Edge>& edges,
                           size_t& next_vert_id, size_t& next_edge_id)
{
    auto& s_pos = edge->vert->position;
    auto& e_pos = edge->next->vert->position;

    auto s_dist = plane.GetDistance(s_pos);
    auto e_dist = plane.GetDistance(e_pos);

    assert(fabs(s_dist) > POINT_STATUS_EPSILON
        && fabs(e_dist) > POINT_STATUS_EPSILON
        && s_dist * e_dist < 0);

    float dot = s_dist / (s_dist - e_dist);
    assert(dot > 0.0f && dot < 1.0f);

    auto pos = s_pos + (e_pos - s_pos) * dot;
    auto new_vert = new he::Vertex(pos, next_vert_id++);
    vertices.Append(new_vert);
    auto new_edge = new he::Edge(new_vert, edge->face, edge->ids);
    new_edge->ids.Append(next_edge_id++);
    edges.Append(new_edge);

    edge->ids.Append(next_edge_id++);
    auto edge_next = edge->next;
    edge->Connect(new_edge)->Connect(edge_next);

    auto twin_edge = edge->twin;
    if (twin_edge)
    {
        auto new_twin_edge = new he::Edge(new_vert, twin_edge->face, twin_edge->ids);
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

void IntersectWithPlane(he::Edge* old_boundary_first,
                        he::Edge* new_boundary_first,
                        he::DoublyLinkedList<he::Edge>& edges,
                        he::DoublyLinkedList<he::Face>& faces,
                        size_t& next_edge_id, size_t& next_face_id)
{
    he::Edge* new_boundary_last = old_boundary_first->prev;

    auto old_face = old_boundary_first->face;
    he::Edge* old_boundary_splitter = new he::Edge(new_boundary_first->vert, old_face, next_edge_id++);
    he::Edge* new_boundary_splitter = new he::Edge(old_boundary_first->vert, old_face, next_edge_id++);
    he::edge_make_pair(old_boundary_splitter, new_boundary_splitter);

    auto new_boundary_first_prev = new_boundary_first->prev;
    old_boundary_first->prev->Connect(new_boundary_splitter);
    new_boundary_splitter->Connect(new_boundary_first);
    new_boundary_first_prev->Connect(old_boundary_splitter);
    old_boundary_splitter->Connect(old_boundary_first);

    auto new_face = new he::Face(new_boundary_first->face->ids);
    new_face->ids.Append(next_face_id++);
    he::bind_edge_face(new_face, new_boundary_first);

    old_face->ids.Append(next_face_id++);
    he::bind_edge_face(old_face, old_boundary_first);

    edges.Append(old_boundary_splitter);
    edges.Append(new_boundary_splitter);
    faces.Append(new_face);
}

he::Edge* IntersectWithPlane(he::Edge* first_boundary_edge, const sm::Plane& plane,
                             he::DoublyLinkedList<he::Vertex>& vertices,
                             he::DoublyLinkedList<he::Edge>& edges,
                             he::DoublyLinkedList<he::Face>& faces,
                             size_t& next_vert_id, size_t& next_edge_id, size_t& next_face_id)
{
    he::Edge* seam_ori = nullptr;
    he::Edge* seam_dst = nullptr;
    he::Edge* seam_dst_prev = nullptr;
    he::Edge* curr_boundary_edge_prev = nullptr;

    he::Edge* curr_boundary_edge = first_boundary_edge;
    do {
        PointStatus os = CalcPointStatus(plane, curr_boundary_edge->vert->position);
        PointStatus ds = CalcPointStatus(plane, curr_boundary_edge->next->vert->position);

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
            SplitEdgeByPlane(curr_boundary_edge, plane, vertices, edges, next_vert_id, next_edge_id);
            curr_boundary_edge = curr_boundary_edge->next;

            auto new_vertex = curr_boundary_edge->vert;
            assert(CalcPointStatus(plane, new_vertex->position) == PointStatus::Inside);
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
        auto os = CalcPointStatus(plane, seam_ori->next->vert->position);
        assert(os != PointStatus::Inside);
        if (os == PointStatus::Below) {
            IntersectWithPlane(seam_ori, seam_dst, edges, faces, next_edge_id, next_face_id);
        } else {
            IntersectWithPlane(seam_dst, seam_ori, edges, faces, next_edge_id, next_face_id);
        }
    }

    return seam_dst->prev;
}

he::Edge* FindNextIntersectingEdge(he::Edge* search_from, const sm::Plane& plane)
{
    auto curr_edge = search_from->next;
    auto stop_edge = search_from->twin;
    do {
        assert(curr_edge != stop_edge);

        auto cd = curr_edge->next->vert;
        auto po = curr_edge->prev->vert;
        auto cds = CalcPointStatus(plane, cd->position);
        auto pos = CalcPointStatus(plane, po->position);
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

std::vector<he::Edge*> IntersectWithPlaneImpl(he::Edge* start_edge, const sm::Plane& plane,
                                              he::DoublyLinkedList<he::Vertex>& vertices,
                                              he::DoublyLinkedList<he::Edge>& edges,
                                              he::DoublyLinkedList<he::Face>& faces,
                                              size_t& next_vert_id, size_t& next_edge_id, size_t& next_face_id)
{
    std::vector<he::Edge*> seam;

    auto curr_edge = start_edge;
    auto stop_vert = curr_edge->next->vert;
    do {
        curr_edge = FindNextIntersectingEdge(curr_edge, plane);
        if (!curr_edge) {
            return std::vector<he::Edge*>();
        }

        curr_edge = IntersectWithPlane(curr_edge, plane, vertices, edges, faces, next_vert_id, next_edge_id, next_face_id);
        seam.push_back(curr_edge);
    } while (curr_edge->next->vert != stop_vert);

    if (seam.empty()) {
        seam.push_back(start_edge);
    }

    return seam;
}

std::vector<he::Edge*> IntersectWithPlane(const sm::Plane& plane,
                                          he::DoublyLinkedList<he::Vertex>& vertices,
                                          he::DoublyLinkedList<he::Edge>& edges,
                                          he::DoublyLinkedList<he::Face>& faces,
                                          size_t& next_vert_id, size_t& next_edge_id, size_t& next_face_id)
{
    std::vector<he::Edge*> seam;

    auto init_edge = FindInitialIntersectingEdge(plane, edges);
    assert(init_edge);

    auto start_edge = IntersectWithPlane(init_edge, plane, vertices, edges, faces, next_vert_id, next_edge_id, next_face_id);
    seam = IntersectWithPlaneImpl(start_edge, plane, vertices, edges, faces, next_vert_id, next_edge_id, next_face_id);
    if (seam.empty()) {
        seam = IntersectWithPlaneImpl(start_edge->twin, plane, vertices, edges, faces, next_vert_id, next_edge_id, next_face_id);
    }

    if (seam.empty()) {
        seam.push_back(start_edge);
    }

    return seam;
}

bool HasMultipleLoops(const std::vector<he::Edge*>& seam)
{
    std::set<he::Vertex*> visited;
    for (auto& e : seam) {
        if (!visited.insert(e->vert).second) {
            return true;
        }
    }
    return false;
}

void OnVertexInvalid(he::DoublyLinkedList<he::Vertex>& vertices,
    he::DoublyLinkedList<he::Edge>& edges, he::DoublyLinkedList<he::Face>& faces);

void OnEdgeInvalid(he::DoublyLinkedList<he::Vertex>& vertices,
                   he::DoublyLinkedList<he::Edge>& edges,
                   he::DoublyLinkedList<he::Face>& faces)
{
    bool edge_dirty = false;

    auto curr_face = faces.Head();
    auto first_face = curr_face;
    do {
        if (curr_face->ids.IsValid())
        {
            auto curr_edge = curr_face->edge;
            auto first_edge = curr_edge;
            do {
                if (!curr_edge->ids.IsValid()) {
                    curr_face->ids.MakeInvalid();
                    break;
                }
                curr_edge = curr_edge->next;
            } while (curr_edge != first_edge);

            if (!curr_face->ids.IsValid())
            {
                auto curr_edge = curr_face->edge;
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

        curr_face = curr_face->linked_next;
    } while (curr_face != first_face);

    if (edge_dirty) {
        OnEdgeInvalid(vertices, edges, faces);
    }
}

void OnVertexInvalid(he::DoublyLinkedList<he::Vertex>& vertices,
                     he::DoublyLinkedList<he::Edge>& edges,
                     he::DoublyLinkedList<he::Face>& faces)
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
        OnEdgeInvalid(vertices, edges, faces);
    }
}

bool FixVertexInvalid(he::DoublyLinkedList<he::Vertex>& vertices,
                      he::DoublyLinkedList<he::Edge>& edges)
{
    bool vert_dirty = false;

    // map vertex to edges
    std::map<he::Vertex*, std::vector<he::Edge*>> vert2edges;
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

    // fix vertices
    auto first_vert = vertices.Head();
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

void FixEdgeInvalid(he::DoublyLinkedList<he::Edge>& edges)
{
    auto first_edge = edges.Head();
    auto curr_edge = first_edge;
    do {
        if (curr_edge->ids.IsValid())
        {
            assert(curr_edge->vert->ids.IsValid());
            assert(curr_edge->face->ids.IsValid());
            assert(curr_edge->prev->ids.IsValid());
            assert(curr_edge->next->ids.IsValid());
            if (curr_edge->twin && !curr_edge->twin->ids.IsValid()) {
                curr_edge->twin = nullptr;
            }
        }

        curr_edge = curr_edge->linked_next;
    } while (curr_edge != first_edge);
}

void FixFaceInvalid(he::DoublyLinkedList<he::Face>& faces)
{
    auto first_face = faces.Head();
    auto curr_face = first_face;
    do {
        if (curr_face->edge->ids.IsValid())
        {
            auto first_edge = curr_face->edge;
            auto curr_edge = first_edge;
            do {
                if (curr_edge->ids.IsValid()) {
                    curr_face->edge = curr_edge;
                    break;
                }

                curr_edge = curr_edge->next;
            } while (curr_edge != first_edge);

            assert(curr_face->edge->ids.IsValid());
        }

        curr_face = curr_face->linked_next;
    } while (curr_face != first_face);
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
                   he::DoublyLinkedList<he::Vertex>& vertices,
                   he::DoublyLinkedList<he::Edge>& edges,
                   he::DoublyLinkedList<he::Face>& faces)
{
    bool vert_dirty = false;

    auto curr_vert = vertices.Head();
    auto first_vert = curr_vert;
    do {
        auto st = CalcPointStatus(plane, curr_vert->position);
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
        OnVertexInvalid(vertices, edges, faces);
    } while (FixVertexInvalid(vertices, edges));
    FixEdgeInvalid(edges);
    FixFaceInvalid(faces);

    DeleteInvalid(vertices);
    DeleteInvalid(edges);
    DeleteInvalid(faces);
}

sm::cube CalcAABB(const he::DoublyLinkedList<he::Vertex>& vertices)
{
    if (vertices.Size() == 0) {
        return sm::cube();
    }

    sm::cube aabb;
    auto first_vertex = vertices.Head();
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
    auto st = CheckIntersects(plane, m_vertices);
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

    auto seam = IntersectWithPlane(plane, m_vertices, m_edges, m_faces,
        m_next_vert_id, m_next_edge_id, m_next_face_id);
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

        auto new_face = new Face(m_next_face_id++);
        m_faces.Append(new_face);

        std::vector<Edge*> new_edges;
        new_edges.reserve(seam.size());
        for (int i = 0, n = seam.size(); i < n; ++i)
        {
            Edge* new_edge = new Edge(seam[(i + 1) % n]->vert, new_face, m_next_edge_id++);
            edge_make_pair(new_edge, seam[i]);
            new_edges.push_back(new_edge);
            m_edges.Append(new_edge);
        }

        for (auto& e : seam) {
            assert(e->next->vert == e->twin->vert);
        }

        for (int i = 0, n = new_edges.size(); i < n; ++i) {
            new_edges[i]->Connect(new_edges[(i - 1 + n) % n]);
        }

        for (auto& e : seam) {
            assert(e->next->vert == e->twin->vert);
        }

        new_face->edge = new_edges[0];
    }

    DeleteByPlane(plane, keep == KeepType::KeepBelow, m_vertices, m_edges, m_faces);

    m_aabb = CalcAABB(m_vertices);

    return true;
}

}