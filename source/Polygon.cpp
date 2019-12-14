#include "halfedge/Polygon.h"
#include "halfedge/Utility.h"

#include <SM_Calc.h>

#include <map>

namespace he
{

size_t Polygon::m_next_vert_id = 0;
size_t Polygon::m_next_edge_id = 0;
size_t Polygon::m_next_loop_id = 0;

Polygon::Polygon(const Polygon& poly)
{
    this->operator = (poly);
}

Polygon::Polygon(const std::vector<std::pair<TopoID, sm::vec2>>& vertices,
                 const std::vector<std::pair<TopoID, std::vector<size_t>>>& faces,
                 const std::vector<std::pair<TopoID, std::vector<size_t>>>& holes)
{
    BuildFromLoops(vertices, faces, holes);
}

Polygon& Polygon::operator = (const Polygon& poly)
{
    std::map<vert2*, size_t> vert2idx;
    auto vertices = DumpVertices(poly.m_verts, vert2idx);

    auto borders = DumpFaces(poly.m_borders, vert2idx);
    auto holes = DumpFaces(poly.m_holes, vert2idx);

    BuildFromLoops(vertices, borders, holes);

//    OffsetTopoID(m_next_vert_id, m_next_edge_id, m_next_loop_id);

    return *this;
}

void Polygon::Clear()
{
    m_next_vert_id = 0;
    m_next_edge_id = 0;
    m_next_loop_id = 0;

    m_verts.Clear();
    m_edges.Clear();
    m_borders.Clear();
    m_holes.Clear();
}

void Polygon::BuildFromLoops(const std::vector<std::pair<TopoID, sm::vec2>>& vertices,
                             const std::vector<std::pair<TopoID, std::vector<size_t>>>& borders,
                             const std::vector<std::pair<TopoID, std::vector<size_t>>>& holes)
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

	for (auto& border : borders)
	{
        auto loop = CreateLoop(v_array, border);
        if (Utility::IsLoopClockwise(*loop)) {
            Utility::FlipFace(*loop);
        }
        m_borders.Append(loop);
	}

    for (auto& hole : holes)
    {
        auto loop = CreateLoop(v_array, hole);
        if (!Utility::IsLoopClockwise(*loop)) {
            Utility::FlipFace(*loop);
        }
        m_holes.Append(loop);
    }
}

face2* Polygon::CreateLoop(const std::vector<vert2*>& verts, const std::pair<TopoID, std::vector<size_t>>& loop)
{
    if (loop.second.size() <= 2) {
        return nullptr;
    }

    TopoID topo_id;
    if (loop.first.Empty()) {
        topo_id = TopoID(m_next_loop_id++);
    } else {
        topo_id = loop.first;
        for (auto& id : loop.first.Path()) {
            if (id >= m_next_loop_id) {
                m_next_loop_id = id + 1;
            }
        }
    }

	auto face = new face2(topo_id);

	assert(loop.second.size() >= 2);
	edge2* first = nullptr;
	edge2* last  = nullptr;

	for (int i = 0, n = loop.second.size(); i < n; ++i)
	{
        auto& curr_pos = loop.second[i];
        auto& next_pos = loop.second[(i + 1) % n];
        assert(curr_pos >= 0 && curr_pos < verts.size());
        auto vert = verts[curr_pos];
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

    return face;
}

std::vector<std::pair<TopoID, sm::vec2>>
Polygon::DumpVertices(const DoublyLinkedList<vert2>& verts, std::map<vert2*, size_t>& vert2idx)
{
    std::vector<std::pair<TopoID, sm::vec2>> vertices;
    vertices.reserve(verts.Size());

    auto first_v = verts.Head();
    auto curr_v = first_v;
    size_t idx = 0;
    do {
        vertices.push_back({ curr_v->ids, curr_v->position });
        vert2idx.insert({ curr_v, idx++ });

        curr_v = curr_v->linked_next;
    } while (curr_v != first_v);

    return vertices;
}

std::vector<std::pair<TopoID, std::vector<size_t>>>
Polygon::DumpFaces(const DoublyLinkedList<face2>& src_faces, const std::map<vert2*, size_t>& vert2idx)
{
    std::vector<std::pair<TopoID, std::vector<size_t>>> faces;
    faces.reserve(src_faces.Size());

    auto first_f = src_faces.Head();
    auto curr_f = first_f;
    do {
        std::vector<size_t> face;

        auto first_e = curr_f->edge;
        auto curr_e = first_e;
        do {
            auto itr = vert2idx.find(curr_e->vert);
            assert(itr != vert2idx.end());
            face.push_back(itr->second);

            curr_e = curr_e->next;
        } while (curr_e && curr_e != first_e);

        faces.push_back({ curr_f->ids, face });

        curr_f = curr_f->linked_next;
    } while (curr_f != first_f);

    return faces;
}

}