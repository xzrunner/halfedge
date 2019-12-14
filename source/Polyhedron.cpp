#include "halfedge/Polyhedron.h"

#include <SM_Vector.h>

#include <map>

namespace he
{

size_t Polyhedron::m_next_vert_id = 0;
size_t Polyhedron::m_next_edge_id = 0;
size_t Polyhedron::m_next_face_id = 0;

Polyhedron::Polyhedron(const Polyhedron& poly)
{
    this->operator = (poly);
}

Polyhedron::Polyhedron(const sm::cube& aabb)
{
    BuildFromCube(aabb);
}

Polyhedron::Polyhedron(const std::vector<std::pair<TopoID, sm::vec3>>& vertices,
                       const std::vector<std::pair<TopoID, std::vector<size_t>>>& faces)
{
    BuildFromPolygons(vertices, faces);
}

Polyhedron& Polyhedron::operator = (const Polyhedron& poly)
{
    std::vector<std::pair<TopoID, sm::vec3>> vertices;
    vertices.reserve(poly.m_verts.Size());

    std::map<vert3*, size_t> vert2idx;

    auto v_head = poly.m_verts.Head();
    if (!v_head) {
        Clear();
        return *this;
    }
    auto v = v_head;
    size_t idx = 0;
    do {
        vertices.push_back({ v->ids, v->position });
        vert2idx.insert({ v, idx++ });
        v = v->linked_next;
    } while (v != v_head);

    std::vector<std::pair<TopoID, std::vector<size_t>>> faces;
    faces.reserve(poly.m_faces.Size());
    auto f_head = poly.m_faces.Head();
    if (!f_head) {
        Clear();
        return *this;
    }
    auto f = f_head;
    do {
        std::vector<size_t> face;

        auto e_head = f->edge;
        auto e = e_head;
        do {
            auto itr = vert2idx.find(e->vert);
            assert(itr != vert2idx.end());
            face.push_back(itr->second);
            e = e->next;
        } while (e != e_head);

        faces.push_back({ f->ids, face });

        f = f->linked_next;
    } while (f != f_head);

    BuildFromFaces(vertices, faces);

    OffsetTopoID(m_next_vert_id, m_next_edge_id, m_next_face_id);

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
    m_next_face_id += f_off;

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

    auto first_f = m_faces.Head();
    auto curr_f = first_f;
    do {
        curr_f->ids.Offset(f_off);
        for (auto& id : curr_f->ids.Path()) {
            if (id >= m_next_face_id) {
                m_next_face_id = id + 1;
            }
        }
        curr_f = curr_f->linked_next;
    } while (curr_f != first_f);
}

void Polyhedron::Clear()
{
    m_next_vert_id = 0;
    m_next_edge_id = 0;
    m_next_face_id = 0;

    m_verts.Clear();
    m_edges.Clear();
    m_faces.Clear();

    m_aabb.MakeEmpty();
}

void Polyhedron::BuildFromCube(const sm::cube& aabb)
{
    Clear();

    m_aabb = aabb;

	sm::vec3 p1(aabb.min[0], aabb.min[1], aabb.min[2]);
	sm::vec3 p2(aabb.min[0], aabb.min[1], aabb.max[2]);
	sm::vec3 p3(aabb.min[0], aabb.max[1], aabb.min[2]);
	sm::vec3 p4(aabb.min[0], aabb.max[1], aabb.max[2]);
	sm::vec3 p5(aabb.max[0], aabb.min[1], aabb.min[2]);
	sm::vec3 p6(aabb.max[0], aabb.min[1], aabb.max[2]);
	sm::vec3 p7(aabb.max[0], aabb.max[1], aabb.min[2]);
	sm::vec3 p8(aabb.max[0], aabb.max[1], aabb.max[2]);

	auto left_bottom_front  = new vert3(p1, m_next_vert_id++);
	auto left_bottom_back   = new vert3(p2, m_next_vert_id++);
	auto left_top_front     = new vert3(p3, m_next_vert_id++);
	auto left_top_back      = new vert3(p4, m_next_vert_id++);
	auto right_bottom_front = new vert3(p5, m_next_vert_id++);
	auto right_bottom_back  = new vert3(p6, m_next_vert_id++);
	auto right_top_front    = new vert3(p7, m_next_vert_id++);
	auto right_top_back     = new vert3(p8, m_next_vert_id++);

    m_vertices.Append(left_bottom_front).Append(left_bottom_back).Append(left_top_front).Append(left_top_back)
              .Append(right_bottom_front).Append(right_bottom_back).Append(right_top_front).Append(right_top_back);

	// Bottom face
	auto bottom = new face3(m_next_face_id++);
	auto bottom_left  = new edge3(left_bottom_front,  bottom, m_next_edge_id++);
	auto bottom_back  = new edge3(left_bottom_back,   bottom, m_next_edge_id++);
	auto bottom_right = new edge3(right_bottom_back,  bottom, m_next_edge_id++);
	auto bottom_front = new edge3(right_bottom_front, bottom, m_next_edge_id++);
    bottom_left->Connect(bottom_back)->Connect(bottom_right)
               ->Connect(bottom_front)->Connect(bottom_left);
	bottom->edge = bottom_left;
    m_edges.Append(bottom_left).Append(bottom_back)
           .Append(bottom_right).Append(bottom_front);
	m_faces.Append(bottom);

	// Left face
	auto left = new face3(m_next_face_id++);
	auto left_bottom = new edge3(left_bottom_back,  left, m_next_edge_id++);
    auto left_front  = new edge3(left_bottom_front, left, m_next_edge_id++);
    auto left_top    = new edge3(left_top_front,    left, m_next_edge_id++);
	auto left_back   = new edge3(left_top_back,     left, m_next_edge_id++);
    left_bottom->Connect(left_front)->Connect(left_top)
               ->Connect(left_back)->Connect(left_bottom);
	left->edge = left_bottom;
    m_edges.Append(left_bottom).Append(left_front)
           .Append(left_top).Append(left_back);
    m_faces.Append(left);

	// Front face
	auto front = new face3(m_next_face_id++);
	auto front_left   = new edge3(left_top_front,     front, m_next_edge_id++);
    auto front_bottom = new edge3(left_bottom_front,  front, m_next_edge_id++);
    auto front_right  = new edge3(right_bottom_front, front, m_next_edge_id++);
	auto front_top    = new edge3(right_top_front,    front, m_next_edge_id++);
    front_left->Connect(front_bottom)->Connect(front_right)
              ->Connect(front_top)->Connect(front_left);
	front->edge = front_left;
    m_edges.Append(front_left).Append(front_bottom)
           .Append(front_right).Append(front_top);
    m_faces.Append(front);

	// Back face
	auto back = new face3(m_next_face_id++);
	auto back_bottom = new edge3(right_bottom_back, back, m_next_edge_id++);
    auto back_left   = new edge3(left_bottom_back,  back, m_next_edge_id++);
    auto back_top    = new edge3(left_top_back,     back, m_next_edge_id++);
	auto back_right  = new edge3(right_top_back,    back, m_next_edge_id++);
    back_bottom->Connect(back_left)->Connect(back_top)
               ->Connect(back_right)->Connect(back_bottom);
	back->edge = back_bottom;
    m_edges.Append(back_bottom).Append(back_left)
           .Append(back_top).Append(back_right);
    m_faces.Append(back);

	// Top face
	auto top = new face3(m_next_face_id++);
	auto top_left  = new edge3(left_top_back,   top, m_next_edge_id++);
    auto top_front = new edge3(left_top_front,  top, m_next_edge_id++);
    auto top_right = new edge3(right_top_front, top, m_next_edge_id++);
	auto top_back  = new edge3(right_top_back,  top, m_next_edge_id++);
    top_left->Connect(top_front)->Connect(top_right)
            ->Connect(top_back)->Connect(top_left);
	top->edge = top_left;
    m_edges.Append(top_left).Append(top_front)
           .Append(top_right).Append(top_back);
    m_faces.Append(top);

	// Right face
	auto right = new face3(m_next_face_id++);
	auto right_front  = new edge3(right_top_front,    right, m_next_edge_id++);
    auto right_bottom = new edge3(right_bottom_front, right, m_next_edge_id++);
    auto right_back   = new edge3(right_bottom_back,  right, m_next_edge_id++);
	auto right_top    = new edge3(right_top_back,     right, m_next_edge_id++);
    right_front->Connect(right_bottom)->Connect(right_back)
               ->Connect(right_top)->Connect(right_front);
	right->edge = right_front;
    m_edges.Append(right_front).Append(right_bottom)
           .Append(right_back).Append(right_top);
    m_faces.Append(right);

    edge_make_pair(top_left,  left_top);
    edge_make_pair(top_back,  back_top);
    edge_make_pair(top_right, right_top);
    edge_make_pair(top_front, front_top);

    edge_make_pair(bottom_left,  left_bottom);
    edge_make_pair(bottom_back,  back_bottom);
    edge_make_pair(bottom_right, right_bottom);
    edge_make_pair(bottom_front, front_bottom);

    edge_make_pair(front_left,  left_front);
    edge_make_pair(front_right, right_front);
    edge_make_pair(back_left,   left_back);
    edge_make_pair(back_right,  right_back);
}

void Polyhedron::BuildFromPolygons(const std::vector<std::pair<TopoID, sm::vec3>>& vertices,
                                   const std::vector<std::pair<TopoID, std::vector<size_t>>>& faces)
{
    Clear();

    std::vector<vert3*> v_array;
    v_array.reserve(vertices.size());
    for (auto& vert : vertices)
    {
        m_aabb.Combine(vert.second);

        TopoID topo_id;
        if (vert.first.Empty()) {
            topo_id = TopoID(m_next_vert_id++);
        } else {
            topo_id = vert.first;
            for (auto& id : vert.first.Path()) {
                if (id >= m_next_vert_id) {
                    m_next_vert_id = id + 1;
                }
            }
        }

        auto v = new vert3(vert.second, topo_id);
        v_array.push_back(v);
        m_vertices.Append(v);
    }

    class EdgeCmp
    {
    public:
        bool operator () (
            const std::pair<size_t, size_t>& e0,
            const std::pair<size_t, size_t>& e1) const {
            return e0.first < e1.first ||
                  (e0.first == e1.first && e0.second < e1.second);
        }
    }; // EdgeCmp
    std::map<std::pair<size_t, size_t>, edge3*, EdgeCmp> map2edge;
	for (auto& src_face : faces)
	{
        if (src_face.second.size() < 2) {
            continue;
        }

        TopoID topo_id;
        if (src_face.first.Empty()) {
            topo_id = TopoID(m_next_face_id++);
        } else {
            topo_id = src_face.first;
            for (auto& id : src_face.first.Path()) {
                if (id >= m_next_face_id) {
                    m_next_face_id = id + 1;
                }
            }
        }

		auto face = new face3(topo_id);

		assert(src_face.second.size() > 2);
		edge3* first = nullptr;
		edge3* last  = nullptr;

		for (int i = 0, n = src_face.second.size(); i < n; ++i)
		{
            auto& curr_pos = src_face.second[i];
            auto& next_pos = src_face.second[(i + 1) % n];

            assert(curr_pos >= 0 && curr_pos < v_array.size());
            auto vert = v_array[curr_pos];
			assert(vert);
			auto edge = new edge3(vert, face, m_next_edge_id++);
            m_edges.Append(edge);
			if (!first) {
				first = edge;
			} else {
                assert(last);
                last->Connect(edge);
			}
			last = edge;

            map2edge.insert({ { curr_pos, next_pos }, edge });
		}
		last->Connect(first);

		face->edge = first;

		m_faces.Append(face);
	}

	// make pair half edge
    for (auto& itr : map2edge)
    {
        if (itr.second->twin) {
            continue;
        }

        auto itr_twin = map2edge.find({ itr.first.second, itr.first.first });
        if (itr_twin != map2edge.end()) {
            edge_make_pair(itr.second, itr_twin->second);
        }
    }
}

}