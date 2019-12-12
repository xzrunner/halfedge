#include "halfedge/Polygon.h"

#include <map>

namespace he
{

size_t Polygon::m_next_vert_id = 0;
size_t Polygon::m_next_edge_id = 0;
size_t Polygon::m_next_face_id = 0;

Polygon::Polygon(const Polygon& poly)
{
    this->operator = (poly);
}

Polygon::Polygon(const std::vector<std::pair<TopoID, sm::vec2>>& vertices,
                 const std::vector<std::pair<TopoID, std::vector<size_t>>>& faces)
{
    BuildFromFaces(vertices, faces);
}

Polygon& Polygon::operator = (const Polygon& poly)
{
    std::vector<std::pair<TopoID, sm::vec2>> vertices;
    vertices.reserve(poly.m_verts.Size());

    std::map<vert2*, size_t> vert2idx;

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
    auto l_head = poly.m_faces.Head();
    if (!l_head) {
        Clear();
        return *this;
    }
    auto l = l_head;
    do {
        std::vector<size_t> face;

        auto e_head = l->edge;
        auto e = e_head;
        do {
            auto itr = vert2idx.find(e->vert);
            assert(itr != vert2idx.end());
            face.push_back(itr->second);
            e = e->next;
        } while (e && e != e_head);

        faces.push_back({ l->ids, face });

        l = l->linked_next;
    } while (l != l_head);

    BuildFromFaces(vertices, faces);

//    OffsetTopoID(m_next_vert_id, m_next_edge_id, m_next_face_id);

    return *this;
}

void Polygon::Clear()
{
    m_next_vert_id = 0;
    m_next_edge_id = 0;
    m_next_face_id = 0;

    m_verts.Clear();
    m_edges.Clear();
    m_faces.Clear();
}

void Polygon::BuildFromFaces(const std::vector<std::pair<TopoID, sm::vec2>>& vertices,
                             const std::vector<std::pair<TopoID, std::vector<size_t>>>& faces)
{
    Clear();

    std::vector<vert2*> v_array;
    v_array.reserve(vertices.size());
    for (auto& vert : vertices)
    {
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

        auto v = new vert2(vert.second, topo_id);
        v_array.push_back(v);
        m_verts.Append(v);
    }

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

		auto face = new face2(topo_id);

		assert(src_face.second.size() >= 2);
		edge2* first = nullptr;
		edge2* last  = nullptr;

		for (int i = 0, n = src_face.second.size(); i < n; ++i)
		{
            auto& curr_pos = src_face.second[i];
            auto& next_pos = src_face.second[(i + 1) % n];
            assert(curr_pos >= 0 && curr_pos < v_array.size());
            auto vert = v_array[curr_pos];
			assert(vert);
			auto edge = new edge2(vert, face, m_next_edge_id++);
            m_edges.Append(edge);
			if (!first) {
				first = edge;
			} else {
                assert(last);
                last->Connect(edge);
			}
			last = edge;
		}
		last->Connect(first);

		face->edge = first;

		m_faces.Append(face);
	}
}

}