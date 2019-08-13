#include "halfedge/Polyhedron.h"

#include <set>

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

bool CheckIntersects(const sm::Plane& plane, const he::DoublyLinkedList<he::Vertex>& vertices)
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

    if (above + inside == sz || below + inside == sz) {
        return false;
    } else {
        return true;
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
            return curr->twin;
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
                           he::DoublyLinkedList<he::Edge>& edges)
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
    auto new_vert = new he::Vertex(pos);
    vertices.Append(new_vert);
    auto new_edge = new he::Edge(new_vert, edge->face);
    edges.Append(new_edge);
    auto edge_next = edge->next;
    edge->Connect(new_edge)->Connect(edge_next);

    auto twin_edge = edge->twin;
    auto new_twin_edge = new he::Edge(new_vert, twin_edge->face);
    edges.Append(new_twin_edge);
    auto twin_edge_next = twin_edge->next;
    twin_edge->Connect(new_twin_edge)->Connect(twin_edge_next);

    edge_make_pair(new_edge, twin_edge);
    edge_make_pair(new_twin_edge, edge);

    return new_edge;
}

void IntersectWithPlane(he::Edge* old_boundary_first,
                        he::Edge* new_boundary_first,
                        he::DoublyLinkedList<he::Edge>& edges,
                        he::DoublyLinkedList<he::Face>& faces)
{
    he::Edge* new_boundary_last = old_boundary_first->prev;

    auto old_face = old_boundary_first->face;
    he::Edge* old_boundary_splitter = new he::Edge(new_boundary_first->vert, old_face);
    he::Edge* new_boundary_splitter = new he::Edge(old_boundary_first->vert, old_face);
    he::edge_make_pair(old_boundary_splitter, new_boundary_splitter);

    auto new_boundary_first_prev = new_boundary_first->prev;
    old_boundary_first->prev->Connect(new_boundary_splitter);
    new_boundary_splitter->Connect(new_boundary_first);
    new_boundary_first_prev->Connect(old_boundary_splitter);
    old_boundary_splitter->Connect(old_boundary_first);

    he::bind_edge_face(old_face, old_boundary_first);

    auto new_face = new he::Face();
    he::bind_edge_face(new_face, new_boundary_first);

    edges.Append(old_boundary_splitter);
    edges.Append(new_boundary_splitter);
    faces.Append(new_face);
}

he::Edge* IntersectWithPlane(he::Edge* first_boundary_edge, const sm::Plane& plane,
                             he::DoublyLinkedList<he::Vertex>& vertices,
                             he::DoublyLinkedList<he::Edge>& edges,
                             he::DoublyLinkedList<he::Face>& faces)
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
            SplitEdgeByPlane(curr_boundary_edge, plane, vertices, edges);
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
            IntersectWithPlane(seam_ori, seam_dst, edges, faces);
        } else {
            IntersectWithPlane(seam_dst, seam_ori, edges, faces);
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

        curr_edge = curr_edge->twin->next;
    } while (curr_edge != stop_edge);

    return nullptr;
}

std::vector<he::Edge*> IntersectWithPlane(const sm::Plane& plane,
                                          he::DoublyLinkedList<he::Vertex>& vertices,
                                          he::DoublyLinkedList<he::Edge>& edges,
                                          he::DoublyLinkedList<he::Face>& faces)
{
    std::vector<he::Edge*> seam;

    auto init_edge = FindInitialIntersectingEdge(plane, edges);
    assert(init_edge);

    auto curr_edge = IntersectWithPlane(init_edge, plane, vertices, edges, faces);
    auto stop_vert = curr_edge->vert;
    do {
        curr_edge = FindNextIntersectingEdge(curr_edge, plane);
        if (!curr_edge) {
            // no seam
            assert(0);
            return std::vector<he::Edge*>();
        }

        curr_edge = IntersectWithPlane(curr_edge, plane, vertices, edges, faces);
        seam.push_back(curr_edge);
    } while (curr_edge->vert != stop_vert);

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

void DeleteByPlane(const sm::Plane& plane, bool del_above,
                   he::DoublyLinkedList<he::Vertex>& vertices,
                   he::DoublyLinkedList<he::Edge>& edges,
                   he::DoublyLinkedList<he::Face>& faces)
{
    // vertices set
    std::set<he::Vertex*> del_vertices;
    auto curr_vert = vertices.Head();
    auto first_vert = curr_vert;
    do {
        auto st = CalcPointStatus(plane, curr_vert->position);
        if ((st == PointStatus::Above && del_above) ||
            (st == PointStatus::Below && !del_above)) {
            del_vertices.insert(curr_vert);
        }

        curr_vert = curr_vert->linked_next;
    } while (curr_vert != first_vert);

    if (del_vertices.empty()) {
        return;
    }

    // del vertices
    curr_vert = vertices.Head();
    first_vert = curr_vert;
    for (auto& v : del_vertices) {
        vertices.Remove(v);
    }

    // del faces
    auto curr_face = faces.Head();
    auto first_face = curr_face;
    std::set<he::Face*> del_faces;
    std::set<he::Edge*> del_edges;
    do {
        auto first_edge = curr_face->edge;
        auto curr_edge = first_edge;
        do {
            auto itr = del_vertices.find(curr_edge->vert);
            if (itr != del_vertices.end())
            {
                auto first_edge = curr_face->edge;
                auto curr_edge = first_edge;
                do {
                    del_edges.insert(curr_edge);
                    curr_edge = curr_edge->next;
                } while (curr_edge != first_edge);

                del_faces.insert(curr_face);
                break;
            }
            curr_edge = curr_edge->next;
        } while (curr_edge != first_edge);
        curr_face = curr_face->linked_next;
    } while (curr_face != first_face);
    for (auto& f : del_faces) {
        faces.Remove(f);
    }

    // del edges
    auto curr_edge = edges.Head();
    auto first_edge = curr_edge;
    do {
        auto itr_s = del_vertices.find(curr_edge->vert);
        auto itr_e = del_vertices.find(curr_edge->next->vert);
        if (itr_s != del_vertices.end() || itr_e != del_vertices.end()) {
            del_edges.insert(curr_edge);
        }
        curr_edge = curr_edge->linked_next;
    } while (curr_edge != first_edge);
    for (auto& e : del_edges) {
        edges.Remove(e);
    }

    // fix vertices
    first_vert = vertices.Head();
    curr_vert = first_vert;
    do {
        auto itr = del_edges.find(curr_vert->edge);
        while (itr != del_edges.end()) {
            curr_vert->edge = curr_vert->edge->twin->next;
            itr = del_edges.find(curr_vert->edge);
        }
        curr_vert = curr_vert->linked_next;
    } while (curr_vert != first_vert);

    // fix edges
    first_edge = edges.Head();
    curr_edge = first_edge;
    do {
        assert(del_vertices.find(curr_edge->vert) == del_vertices.end());
        assert(del_faces.find(curr_edge->face) == del_faces.end());
        assert(del_edges.find(curr_edge->prev) == del_edges.end());
        assert(del_edges.find(curr_edge->next) == del_edges.end());
        if (del_edges.find(curr_edge->twin) != del_edges.end()) {
            curr_edge->twin = nullptr;
        }
        curr_edge = curr_edge->linked_next;
    } while (curr_edge != first_edge);

    // fix faces
    first_face = faces.Head();
    curr_face = first_face;
    do {
        auto itr = del_edges.find(curr_face->edge);
        assert(itr == del_edges.end());
        curr_face = curr_face->linked_next;
    } while (curr_face != first_face);

    // delete
    for (auto& v : del_vertices) {
        delete v;
    }
    for (auto& e : del_edges) {
        delete e;
    }
    for (auto& f : del_faces) {
        delete f;
    }
}

sm::cube CalcAABB(const he::DoublyLinkedList<he::Vertex>& vertices)
{
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
    if (!CheckIntersects(plane, m_vertices)) {
        return false;
    }

    auto seam = IntersectWithPlane(plane, m_vertices, m_edges, m_faces);
    if (seam.empty()) {
        return false;
    }

    if (keep == KeepType::KeepAll) {
        return true;
    }

    if (seam_face && keep == KeepType::KeepBelow) {
        for (auto& s : seam) {
            s = s->twin;
        }
    }

    DeleteByPlane(plane, keep == KeepType::KeepBelow, m_vertices, m_edges, m_faces);

    m_aabb = CalcAABB(m_vertices);

    if (seam_face)
    {
        assert(!seam.front()->twin);

        auto new_face = new Face();
        m_faces.Append(new_face);

        std::vector<Edge*> new_edges;
        new_edges.reserve(seam.size());
        for (int i = 0, n = seam.size(); i < n; ++i)
        {
            Edge* new_edge = new Edge(seam[(i + 1) % n]->vert, new_face);
            edge_make_pair(new_edge, seam[i]);
            new_edges.push_back(new_edge);
            m_edges.Append(new_edge);
        }
        for (int i = 0, n = new_edges.size(); i < n; ++i) {
            new_edges[(i + 1) % n]->Connect(new_edges[i]);
        }

        new_face->edge = new_edges[0];
    }

    return true;
}

}