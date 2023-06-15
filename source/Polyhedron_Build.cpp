#include "halfedge/Polyhedron.h"

namespace he
{

vert3* Polyhedron::AddVertex(const sm::vec3& pos)
{
    m_aabb.Combine(pos);

    auto vert = new he::vert3(pos, he::TopoID());
    m_verts.Append(vert);

    return vert;
}

loop3* Polyhedron::AddFace(const std::vector<size_t>& loop_indices, const std::vector<vert3*>& v_array, LoopBuilder& builder)
{
    he::Polyhedron::Face face;

    auto loop = BuildLoop(he::TopoID(), loop_indices, v_array, builder);
    m_loops.Append(loop);
    face.border = loop;
    
    m_faces.push_back(face);

    return loop;
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

    m_verts.Append(left_bottom_front).Append(left_bottom_back).Append(left_top_front).Append(left_top_back)
              .Append(right_bottom_front).Append(right_bottom_back).Append(right_top_front).Append(right_top_back);

	// Bottom face
	auto bottom = new loop3(m_next_loop_id++);
	auto bottom_left  = new edge3(left_bottom_front,  bottom, m_next_edge_id++);
	auto bottom_back  = new edge3(left_bottom_back,   bottom, m_next_edge_id++);
	auto bottom_right = new edge3(right_bottom_back,  bottom, m_next_edge_id++);
	auto bottom_front = new edge3(right_bottom_front, bottom, m_next_edge_id++);
    bottom_left->Connect(bottom_back)->Connect(bottom_right)
               ->Connect(bottom_front)->Connect(bottom_left);
	bottom->edge = bottom_left;
    m_edges.Append(bottom_left).Append(bottom_back)
           .Append(bottom_right).Append(bottom_front);
	m_loops.Append(bottom);
    m_faces.emplace_back(bottom);

	// Left face
	auto left = new loop3(m_next_loop_id++);
	auto left_bottom = new edge3(left_bottom_back,  left, m_next_edge_id++);
    auto left_front  = new edge3(left_bottom_front, left, m_next_edge_id++);
    auto left_top    = new edge3(left_top_front,    left, m_next_edge_id++);
	auto left_back   = new edge3(left_top_back,     left, m_next_edge_id++);
    left_bottom->Connect(left_front)->Connect(left_top)
               ->Connect(left_back)->Connect(left_bottom);
	left->edge = left_bottom;
    m_edges.Append(left_bottom).Append(left_front)
           .Append(left_top).Append(left_back);
    m_loops.Append(left);
    m_faces.emplace_back(left);

	// Front face
	auto front = new loop3(m_next_loop_id++);
	auto front_left   = new edge3(left_top_front,     front, m_next_edge_id++);
    auto front_bottom = new edge3(left_bottom_front,  front, m_next_edge_id++);
    auto front_right  = new edge3(right_bottom_front, front, m_next_edge_id++);
	auto front_top    = new edge3(right_top_front,    front, m_next_edge_id++);
    front_left->Connect(front_bottom)->Connect(front_right)
              ->Connect(front_top)->Connect(front_left);
	front->edge = front_left;
    m_edges.Append(front_left).Append(front_bottom)
           .Append(front_right).Append(front_top);
    m_loops.Append(front);
    m_faces.emplace_back(front);

	// Back face
	auto back = new loop3(m_next_loop_id++);
	auto back_bottom = new edge3(right_bottom_back, back, m_next_edge_id++);
    auto back_left   = new edge3(left_bottom_back,  back, m_next_edge_id++);
    auto back_top    = new edge3(left_top_back,     back, m_next_edge_id++);
	auto back_right  = new edge3(right_top_back,    back, m_next_edge_id++);
    back_bottom->Connect(back_left)->Connect(back_top)
               ->Connect(back_right)->Connect(back_bottom);
	back->edge = back_bottom;
    m_edges.Append(back_bottom).Append(back_left)
           .Append(back_top).Append(back_right);
    m_loops.Append(back);
    m_faces.emplace_back(back);

	// Top face
	auto top = new loop3(m_next_loop_id++);
	auto top_left  = new edge3(left_top_back,   top, m_next_edge_id++);
    auto top_front = new edge3(left_top_front,  top, m_next_edge_id++);
    auto top_right = new edge3(right_top_front, top, m_next_edge_id++);
	auto top_back  = new edge3(right_top_back,  top, m_next_edge_id++);
    top_left->Connect(top_front)->Connect(top_right)
            ->Connect(top_back)->Connect(top_left);
	top->edge = top_left;
    m_edges.Append(top_left).Append(top_front)
           .Append(top_right).Append(top_back);
    m_loops.Append(top);
    m_faces.emplace_back(top);

	// Right face
	auto right = new loop3(m_next_loop_id++);
	auto right_front  = new edge3(right_top_front,    right, m_next_edge_id++);
    auto right_bottom = new edge3(right_bottom_front, right, m_next_edge_id++);
    auto right_back   = new edge3(right_bottom_back,  right, m_next_edge_id++);
	auto right_top    = new edge3(right_top_back,     right, m_next_edge_id++);
    right_front->Connect(right_bottom)->Connect(right_back)
               ->Connect(right_top)->Connect(right_front);
	right->edge = right_front;
    m_edges.Append(right_front).Append(right_bottom)
           .Append(right_back).Append(right_top);
    m_loops.Append(right);
    m_faces.emplace_back(right);

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

void Polyhedron::BuildFromFaces(const std::vector<in_vert>& verts,
                                const std::vector<in_face>& faces)
{
    Clear();

    std::vector<vert3*> v_array;
    BuildVertices(verts, v_array);

    LoopBuilder builder;
	for (auto& face : faces)
	{
        Face dst_f;

        auto& id     = std::get<0>(face);
        auto& border = std::get<1>(face);
        auto& holes  = std::get<2>(face);

        auto border_loop = BuildLoop(id, border, v_array, builder);
        assert(border_loop);
        m_loops.Append(border_loop);
        dst_f.border = border_loop;

        dst_f.holes.reserve(holes.size());
        for (auto& hole : holes)
        {
            auto hole_loop = BuildLoop(id, hole, v_array, builder);
            assert(hole_loop);
            m_loops.Append(hole_loop);
            dst_f.holes.push_back(hole_loop);
        }

        m_faces.push_back(dst_f);
	}

    builder.Build();
}

void Polyhedron::BuildFromFaces(const std::vector<Face>& faces)
{
    std::vector<in_vert> dst_verts;

    std::map<vert3*, size_t> vert2id;
    auto build_verts = [&](const loop3& loop)
    {
        auto first_e = loop.edge;
        auto curr_e = first_e;
        do {
            auto& vert = curr_e->vert;
            auto itr = vert2id.find(vert);
            if (itr == vert2id.end())
            {
                size_t id = dst_verts.size();
                dst_verts.push_back({ vert->ids, vert->position });
                vert2id.insert({ vert, id });
            }

            curr_e = curr_e->next;
        } while (curr_e != first_e);
    };

    for (auto& face : faces)
    {
        if (face.border) {
            build_verts(*face.border);
        }
        for (auto& hole : face.holes) {
            build_verts(*hole);
        }
    }

    auto build_loops = [](const loop3& loop, const std::map<vert3*, size_t>& vert2id) -> in_loop
    {
        in_loop dst;

        auto first_e = loop.edge;
        auto curr_e = first_e;
        do {
            auto& vert = curr_e->vert;
            auto itr = vert2id.find(vert);
            assert(itr != vert2id.end());
            dst.push_back(itr->second);

            curr_e = curr_e->next;
        } while (curr_e != first_e);

        return dst;
    };

    std::vector<in_face> dst_faces;
    for (auto& face : faces)
    {
        in_face dst_face;
        std::get<0>(dst_face) = face.border->ids;
        if (face.border) {
            std::get<1>(dst_face) = build_loops(*face.border, vert2id);
        }
        for (auto& hole : face.holes) {
            std::get<2>(dst_face).push_back(build_loops(*hole, vert2id));
        }
        dst_faces.push_back(dst_face);
    }

    BuildFromFaces(dst_verts, dst_faces);
}

void Polyhedron::BuildVertices(const std::vector<in_vert>& verts, std::vector<vert3*>& v_array)
{
    v_array.reserve(verts.size());
    for (auto& vert : verts)
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
        m_verts.Append(v);
    }
}

loop3* Polyhedron::BuildLoop(TopoID id, const std::vector<size_t>& loop, const std::vector<vert3*>& v_array, LoopBuilder& builder)
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

	auto ret = new loop3(topo_id);

	assert(loop.size() > 2);
	edge3* first = nullptr;
	edge3* last  = nullptr;

	for (int i = 0, n = loop.size(); i < n; ++i)
	{
        auto& curr_pos = loop[i];
        auto& next_pos = loop[(i + 1) % n];

        assert(curr_pos >= 0 && curr_pos < v_array.size());
        auto vert = v_array[curr_pos];
		assert(vert);
		auto edge = new edge3(vert, ret, m_next_edge_id++);
        m_edges.Append(edge);
		if (!first) {
			first = edge;
		} else {
            assert(last);
            last->Connect(edge);
		}
		last = edge;

        builder.Add({ curr_pos, next_pos }, edge);
	}
	last->Connect(first);

	ret->edge = first;

    return ret;
}

//////////////////////////////////////////////////////////////////////////
// class Polyhedron::LoopBuilder
//////////////////////////////////////////////////////////////////////////

void Polyhedron::LoopBuilder::Build()
{
    for (auto& itr : map2edge)
    {
        if (itr.second->twin) {
            continue;
        }

        auto itr_twin = map2edge.find({ itr.first.second, itr.first.first });
        if (itr_twin != map2edge.end()) {
            he::edge_make_pair(itr.second, itr_twin->second);
        }
    }
}

}