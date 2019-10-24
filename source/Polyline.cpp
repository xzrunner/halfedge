#include "halfedge/Polyline.h"
#include "halfedge/EditHelper.h"

#include <SM_Calc.h>

#include <map>

namespace he
{

size_t Polyline::m_next_vert_id     = 0;
size_t Polyline::m_next_edge_id     = 0;
size_t Polyline::m_next_polyline_id = 0;

Polyline::Polyline(const Polyline& poly)
{
    this->operator = (poly);
}

Polyline::Polyline(const std::vector<std::pair<TopoID, sm::vec3>>& vertices,
                   const std::vector<std::pair<TopoID, std::vector<size_t>>>& polylines)
{
    BuildFromPolylines(vertices, polylines);
}

Polyline& Polyline::operator = (const Polyline& poly)
{
    std::vector<std::pair<TopoID, sm::vec3>> vertices;
    vertices.reserve(poly.m_vertices.Size());

    std::map<Vertex*, size_t> vert2idx;

    auto v_head = poly.m_vertices.Head();
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

    std::vector<std::pair<TopoID, std::vector<size_t>>> polylines;
    polylines.reserve(poly.m_polylines.Size());
    auto l_head = poly.m_polylines.Head();
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

        polylines.push_back({ l->ids, face });

        l = l->linked_next;
    } while (l != l_head);

    BuildFromPolylines(vertices, polylines);

//    OffsetTopoID(m_next_vert_id, m_next_edge_id, m_next_polyline_id);

    return *this;
}

void Polyline::Fuse(float distance)
{
    if (m_polylines.Size() == 0) {
        return;
    }

    std::vector<he::Vertex*> del_vts;

    // invalid flags
    auto first_line = m_polylines.Head();
    auto curr_line = first_line;
    do {
        he::Vertex* prev_vert = nullptr;

        auto first_edge = curr_line->edge;
        auto curr_edge = first_edge;
        do {
            if (prev_vert)
            {
                auto curr_vert = curr_edge->vert;
                auto dist = sm::dis_pos3_to_pos3(prev_vert->position, curr_vert->position);
                if (dist < distance) {
                    curr_vert->ids.MakeInvalid();
                    del_vts.push_back(curr_vert);
                } else {
                    prev_vert = curr_vert;
                }
            }
            else
            {
                prev_vert = curr_edge->vert;
            }

            curr_edge = curr_edge->next;
        } while (curr_edge && curr_edge != first_edge);

        curr_line = curr_line->linked_next;
    } while (curr_line != first_line);

    // update
    first_line = m_polylines.Head();
    curr_line = first_line;
    do {
        he::Vertex* prev_vert = nullptr;

        auto first_edge = curr_line->edge;
        auto curr_edge = first_edge;
        he::Edge* prev_edge = nullptr;
        do {
            if (!curr_edge->vert->ids.IsValid())
            {
                if (curr_edge == curr_line->edge) {
                    curr_line->edge = curr_edge->next;
                }
                if (prev_edge) {
                    prev_edge->Connect(curr_edge->next);
                }
            } else {
                prev_edge = curr_edge;
            }

            curr_edge = curr_edge->next;
        } while (curr_edge && curr_edge != first_edge);

        curr_line = curr_line->linked_next;
    } while (curr_line != first_line);

    // delete
    for (auto& v : del_vts)
    {
        m_edges.Remove(v->edge);
        delete v->edge;
        m_vertices.Remove(v);
        delete v;
    }
}

void Polyline::UniquePoints()
{
    EditHelper::UniquePoints(m_vertices, m_edges, m_next_vert_id);
}

void Polyline::OffsetTopoID(size_t v_off, size_t e_off, size_t f_off)
{
    m_next_vert_id += v_off;
    m_next_edge_id += e_off;
    m_next_polyline_id += f_off;

    auto first_v = m_vertices.Head();
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
    if (first_e)
    {
        auto curr_e = first_e;
        do {
            curr_e->ids.Offset(e_off);
            for (auto& id : curr_e->ids.Path()) {
                if (id >= m_next_edge_id) {
                    m_next_edge_id = id + 1;
                }
            }
            curr_e = curr_e->linked_next;
        } while (curr_e && curr_e != first_e);
    }

    auto first_l = m_polylines.Head();
    if (first_l)
    {
        auto curr_l = first_l;
        do {
            curr_l->ids.Offset(f_off);
            for (auto& id : curr_l->ids.Path()) {
                if (id >= m_next_polyline_id) {
                    m_next_polyline_id = id + 1;
                }
            }
            curr_l = curr_l->linked_next;
        } while (curr_l != first_l);
    }
}

void Polyline::Clear()
{
    m_next_vert_id = 0;
    m_next_edge_id = 0;
    m_next_polyline_id = 0;

    m_vertices.Clear();
    m_edges.Clear();
    m_polylines.Clear();
}

void Polyline::BuildFromPolylines(const std::vector<std::pair<TopoID, sm::vec3>>& vertices,
                                  const std::vector<std::pair<TopoID, std::vector<size_t>>>& polylines)
{
    Clear();

    std::vector<Vertex*> v_array;
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

        auto v = new Vertex(vert.second, topo_id);
        v_array.push_back(v);
        m_vertices.Append(v);
    }

	for (auto& src_polyline : polylines)
	{
        if (src_polyline.second.size() < 2) {
            continue;
        }

        TopoID topo_id;
        if (src_polyline.first.Empty()) {
            topo_id = TopoID(m_next_polyline_id++);
        } else {
            topo_id = src_polyline.first;
            for (auto& id : src_polyline.first.Path()) {
                if (id >= m_next_polyline_id) {
                    m_next_polyline_id = id + 1;
                }
            }
        }

		auto polyline = new Face(topo_id);

		assert(src_polyline.second.size() >= 2);
		Edge* first = nullptr;
		Edge* last  = nullptr;

		for (int i = 0, n = src_polyline.second.size(); i < n; ++i)
		{
            auto& curr_pos = src_polyline.second[i];
            auto& next_pos = src_polyline.second[(i + 1) % n];

            if (curr_pos >= v_array.size()) {
                int zz = 0;
            }

            assert(curr_pos >= 0 && curr_pos < v_array.size());
            auto vert = v_array[curr_pos];
			assert(vert);
			auto edge = new Edge(vert, polyline, m_next_edge_id++);
            m_edges.Append(edge);
			if (!first) {
				first = edge;
			} else {
                assert(last);
                last->Connect(edge);
			}
			last = edge;
		}
//		last->Connect(first);

		polyline->edge = first;

		m_polylines.Append(polyline);
	}
}

}