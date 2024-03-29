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

Polyhedron::Polyhedron(const std::vector<in_vert>& verts, const std::vector<in_face>& faces)
{
    BuildFromFaces(verts, faces);
}

Polyhedron::Polyhedron(const std::vector<Face>& faces)
{
    BuildFromFaces(faces);
}

Polyhedron& Polyhedron::operator = (const Polyhedron& poly)
{
    std::map<vert3*, size_t> vert2idx;
    auto verts = DumpVertices(poly.m_verts, vert2idx);

    std::vector<in_face> faces;
    faces.resize(poly.m_faces.size());
    size_t idx = 0;
    for (auto& face : poly.m_faces)
    {
        std::vector<in_loop> holes;
        holes.reserve(face.holes.size());
        for (auto& hole : face.holes) {
            holes.push_back(DumpLoop(*hole, vert2idx));
        }

        auto& dst = faces[idx++];
        std::get<0>(dst) = face.border->ids;
        std::get<1>(dst) = DumpLoop(*face.border, vert2idx);
        std::get<2>(dst) = holes;
    }

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

    m_faces.clear();

    m_verts.Clear();
    m_edges.Clear();
    m_loops.Clear();

    m_aabb.MakeEmpty();
}

std::vector<Polyhedron::in_vert>
Polyhedron::DumpVertices(const DoublyLinkedList<vert3>& verts, std::map<vert3*, size_t>& vert2idx)
{
    std::vector<std::pair<TopoID, sm::vec3>> ret;
    ret.reserve(verts.Size());

    auto first_v = verts.Head();
    auto curr_v = first_v;
    size_t idx = 0;
    do {
        ret.push_back({ curr_v->ids, curr_v->position });
        vert2idx.insert({ curr_v, idx++ });

        curr_v = curr_v->linked_next;
    } while (curr_v != first_v);

    return ret;
}

Polyhedron::in_loop
Polyhedron::DumpLoop(const loop3& loop, const std::map<vert3*, size_t>& vert2idx)
{
    in_loop ret;

    auto first_e = loop.edge;
    auto curr_e = first_e;
    do {
        auto itr = vert2idx.find(curr_e->vert);
        assert(itr != vert2idx.end());
        ret.push_back(itr->second);

        curr_e = curr_e->next;
    } while (curr_e && curr_e != first_e);

    return ret;
}

}