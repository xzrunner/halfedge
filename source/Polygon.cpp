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

Polygon::Polygon(const std::vector<in_vert>& verts, const std::vector<in_face>& faces)
{
    BuildFromLoops(verts, faces);
}

Polygon& Polygon::operator = (const Polygon& poly)
{
    std::map<vert2*, size_t> vert2idx;
    auto verts = DumpVertices(poly.m_verts, vert2idx);

    std::vector<in_face> faces;
    faces.resize(m_faces.size());
    size_t idx = 0;
    for (auto& face : m_faces)
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

    BuildFromLoops(verts, faces);

//    OffsetTopoID(m_next_vert_id, m_next_edge_id, m_next_loop_id);

    return *this;
}

void Polygon::Clear()
{
    m_next_vert_id = 0;
    m_next_edge_id = 0;
    m_next_loop_id = 0;

    m_faces.clear();

    m_verts.Clear();
    m_edges.Clear();
    m_loops.Clear();
}

void Polygon::BuildFromLoops(const std::vector<in_vert>& verts, const std::vector<in_face>& faces)
{
    Clear();

    std::vector<vert2*> v_array;
    v_array.reserve(verts.size());
    for (auto& vert : verts)
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

    m_faces.reserve(faces.size());
    for (auto& face : faces)
    {
        auto& id     = std::get<0>(face);
        auto& border = std::get<1>(face);
        auto& holes  = std::get<2>(face);

        Face dst_f;

        auto loop = CreateLoop(v_array, id, border);
        if (Utility::IsLoopClockwise(*loop)) {
            Utility::FlipLoop(*loop);
        }
        m_loops.Append(loop);
        dst_f.border = loop;

        dst_f.holes.reserve(holes.size());
        for (auto& hole : holes)
        {
            auto loop = CreateLoop(v_array, id, hole);
            if (!Utility::IsLoopClockwise(*loop)) {
                Utility::FlipLoop(*loop);
            }
            m_loops.Append(loop);
            dst_f.holes.push_back(loop);
        }

        m_faces.push_back(dst_f);
    }
}

loop2* Polygon::CreateLoop(const std::vector<vert2*>& verts, TopoID id, const std::vector<size_t>& loop)
{
    if (loop.size() <= 2) {
        return nullptr;
    }

    TopoID topo_id;
    if (id.Empty()) {
        topo_id = TopoID(m_next_loop_id++);
    } else {
        topo_id = id;
        for (auto& id : id.Path()) {
            if (id >= m_next_loop_id) {
                m_next_loop_id = id + 1;
            }
        }
    }

	auto ret = new loop2(topo_id);

	assert(loop.size() >= 2);
	edge2* first = nullptr;
	edge2* last  = nullptr;

	for (int i = 0, n = loop.size(); i < n; ++i)
	{
        auto& curr_pos = loop[i];
        auto& next_pos = loop[(i + 1) % n];
        assert(curr_pos >= 0 && curr_pos < verts.size());
        auto vert = verts[curr_pos];
		assert(vert);
		auto edge = new edge2(vert, ret, m_next_edge_id++);
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

    ret->edge = first;

    return ret;
}

std::vector<Polygon::in_vert>
Polygon::DumpVertices(const DoublyLinkedList<vert2>& verts, std::map<vert2*, size_t>& vert2idx)
{
    std::vector<std::pair<TopoID, sm::vec2>> ret;
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

Polygon::in_loop Polygon::DumpLoop(const loop2& loop, const std::map<vert2*, size_t>& vert2idx)
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