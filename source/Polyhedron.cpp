#include "halfedge/Polyhedron.h"

#include <SM_Vector.h>

#include <map>

namespace he
{

size_t Polyhedron::m_next_vert_id = 0;
size_t Polyhedron::m_next_edge_id = 0;
size_t Polyhedron::m_next_loop_id = 0;

Polyhedron::Polyhedron(const Polyhedron& poly)
{
    this->operator = (poly);
}

Polyhedron::Polyhedron(const sm::cube& aabb)
{
    BuildFromCube(aabb);
}

Polyhedron::Polyhedron(const std::vector<in_vert>& verts, const std::vector<in_face_no_hole>& faces)
{
    BuildFromFaces(verts, faces);
}

Polyhedron::Polyhedron(const std::vector<in_vert>& verts, const std::vector<in_face_with_hole>& faces)
{
    BuildFromFaces(verts, faces);
}

Polyhedron& Polyhedron::operator = (const Polyhedron& poly)
{
    std::vector<std::pair<TopoID, sm::vec3>> verts;
    verts.reserve(poly.m_verts.Size());

    std::map<vert3*, size_t> vert2idx;

    auto v_head = poly.m_verts.Head();
    if (!v_head) {
        Clear();
        return *this;
    }
    auto v = v_head;
    size_t idx = 0;
    do {
        verts.push_back({ v->ids, v->position });
        vert2idx.insert({ v, idx++ });
        v = v->linked_next;
    } while (v != v_head);

    std::vector<std::pair<TopoID, std::vector<size_t>>> faces;
    faces.reserve(poly.m_loops.Size());
    auto f_head = poly.m_loops.Head();
    if (!f_head) {
        Clear();
        return *this;
    }
    auto f = f_head;
    do {
        std::vector<size_t> loop;

        auto e_head = f->edge;
        auto e = e_head;
        do {
            auto itr = vert2idx.find(e->vert);
            assert(itr != vert2idx.end());
            loop.push_back(itr->second);
            e = e->next;
        } while (e != e_head);

        faces.push_back({ f->ids, loop });

        f = f->linked_next;
    } while (f != f_head);

    BuildFromFaces(verts, faces);

    OffsetTopoID(m_next_vert_id, m_next_edge_id, m_next_loop_id);

    return *this;
}

void Polyhedron::UpdateAABB()
{
	m_aabb.MakeEmpty();

    auto head = m_verts.Head();
    if (head) {
        auto v = head;
        do {
            m_aabb.Combine(v->position);
            v = v->linked_next;
        } while (v != head);
    }
}

void Polyhedron::OffsetTopoID(size_t v_off, size_t e_off, size_t f_off)
{
    m_next_vert_id += v_off;
    m_next_edge_id += e_off;
    m_next_loop_id += f_off;

    auto first_v = m_verts.Head();
    auto curr_v = first_v;
    do {
        curr_v->ids.Offset(v_off);
        for (auto& id : curr_v->ids.Path()) {
            if (id >= m_next_vert_id) {
                m_next_vert_id = id + 1;
            }
        }
        curr_v = curr_v->linked_next;
    } while (curr_v != first_v);

    auto first_e = m_edges.Head();
    auto curr_e = first_e;
    do {
        curr_e->ids.Offset(e_off);
        for (auto& id : curr_e->ids.Path()) {
            if (id >= m_next_edge_id) {
                m_next_edge_id = id + 1;
            }
        }
        curr_e = curr_e->linked_next;
    } while (curr_e != first_e);

    auto first_l = m_loops.Head();
    auto curr_l = first_l;
    do {
        curr_l->ids.Offset(f_off);
        for (auto& id : curr_l->ids.Path()) {
            if (id >= m_next_loop_id) {
                m_next_loop_id = id + 1;
            }
        }
        curr_l = curr_l->linked_next;
    } while (curr_l != first_l);
}

void Polyhedron::Clear()
{
    m_next_vert_id = 0;
    m_next_edge_id = 0;
    m_next_loop_id = 0;

    m_verts.Clear();
    m_edges.Clear();
    m_loops.Clear();

    m_aabb.MakeEmpty();
}

}