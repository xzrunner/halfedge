#include "halfedge/Polyhedron.h"
#include "halfedge/HalfEdge.h"

#include <SM_Vector.h>

#include <map>

namespace he
{

Polyhedron::Polyhedron(const sm::cube& aabb)
	: m_aabb(aabb)
{
	sm::vec3 p1(aabb.min[0], aabb.min[1], aabb.min[2]);
	sm::vec3 p2(aabb.min[0], aabb.min[1], aabb.max[2]);
	sm::vec3 p3(aabb.min[0], aabb.max[1], aabb.min[2]);
	sm::vec3 p4(aabb.min[0], aabb.max[1], aabb.max[2]);
	sm::vec3 p5(aabb.max[0], aabb.min[1], aabb.min[2]);
	sm::vec3 p6(aabb.max[0], aabb.min[1], aabb.max[2]);
	sm::vec3 p7(aabb.max[0], aabb.max[1], aabb.min[2]);
	sm::vec3 p8(aabb.max[0], aabb.max[1], aabb.max[2]);

	auto left_bottom_front  = new Vertex(p1);
	auto left_bottom_back   = new Vertex(p2);
	auto left_top_front     = new Vertex(p3);
	auto left_top_back      = new Vertex(p4);
	auto right_bottom_front = new Vertex(p5);
	auto right_bottom_back  = new Vertex(p6);
	auto right_top_front    = new Vertex(p7);
	auto right_top_back     = new Vertex(p8);

    m_vertices.Append(left_bottom_front).Append(left_bottom_back).Append(left_top_front).Append(left_top_back)
              .Append(right_bottom_front).Append(right_bottom_back).Append(right_top_front).Append(right_top_back);

	// Bottom face
	auto bottom = new Face();
	auto bottom_left  = new Edge(left_bottom_back,   bottom);
	auto bottom_back  = new Edge(right_bottom_back,  bottom);
	auto bottom_right = new Edge(right_bottom_front, bottom);
	auto bottom_front = new Edge(left_bottom_front,  bottom);
    bottom_left->Connect(bottom_back)->Connect(bottom_right)
               ->Connect(bottom_front)->Connect(bottom_left);
	bottom->edge = bottom_left;
    m_edges.Append(bottom_left).Append(bottom_back)
           .Append(bottom_right).Append(bottom_front);
	m_faces.Append(bottom);

	// Left face
	auto left = new Face();
	auto left_bottom = new Edge(left_bottom_front, left);
	auto left_back   = new Edge(left_bottom_back,  left);
	auto left_top    = new Edge(left_top_back,     left);
	auto left_front  = new Edge(left_top_front,    left);
    left_bottom->Connect(left_back)->Connect(left_top)
               ->Connect(left_front)->Connect(left_bottom);
	left->edge = left_bottom;
    m_edges.Append(left_bottom).Append(left_back)
           .Append(left_top).Append(left_front);
    m_faces.Append(left);

	// Front face
	auto front = new Face();
	auto front_left   = new Edge(left_bottom_front,  front);
	auto front_top    = new Edge(left_top_front,     front);
	auto front_right  = new Edge(right_top_front,    front);
	auto front_bottom = new Edge(right_bottom_front, front);
    front_left->Connect(front_top)->Connect(front_right)
              ->Connect(front_bottom)->Connect(front_left);
	front->edge = front_left;
    m_edges.Append(front_left).Append(front_top)
           .Append(front_right).Append(front_bottom);
    m_faces.Append(front);

	// Back face
	auto back = new Face();
	auto back_bottom = new Edge(left_bottom_back,  back);
	auto back_right  = new Edge(right_bottom_back, back);
	auto back_top    = new Edge(right_top_back,    back);
	auto back_left   = new Edge(left_top_back,     back);
    back_bottom->Connect(back_right)->Connect(back_top)
               ->Connect(back_left)->Connect(back_bottom);
	back->edge = back_bottom;
    m_edges.Append(back_bottom).Append(back_right)
           .Append(back_top).Append(back_left);
    m_faces.Append(back);

	// Top face
	auto top = new Face();
	auto top_left  = new Edge(left_top_front,  top);
	auto top_back  = new Edge(left_top_back,   top);
	auto top_right = new Edge(right_top_back,  top);
	auto top_front = new Edge(right_top_front, top);
    top_left->Connect(top_back)->Connect(top_right)
            ->Connect(top_front)->Connect(top_left);
	top->edge = top_left;
    m_edges.Append(top_left).Append(top_back)
           .Append(top_right).Append(top_front);
    m_faces.Append(top);

	// Right face
	auto right = new Face();
	auto right_front  = new Edge(right_bottom_front, right);
	auto right_top    = new Edge(right_top_front, right);
	auto right_back   = new Edge(right_top_back, right);
	auto right_bottom = new Edge(right_bottom_back, right);
    right_front->Connect(right_top)->Connect(right_back)
               ->Connect(right_bottom)->Connect(right_front);
	right->edge = right_front;
    m_edges.Append(right_front).Append(right_top)
           .Append(right_back).Append(right_bottom);
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

Polyhedron::Polyhedron(const std::vector<std::vector<sm::vec3>>& faces_pos)
{
	std::map<sm::vec3, Vertex*, sm::Vector3Cmp> map2vert;
	for (auto& face : faces_pos) {
		for (auto& pos : face) {
			m_aabb.Combine(pos);
			map2vert.insert({ pos, new Vertex(pos) });
		}
	}
	for (auto& itr : map2vert) {
		m_vertices.Append(itr.second);
	}

    class EdgeCmp
    {
    public:
        bool operator () (
            const std::pair<sm::vec3, sm::vec3>& e0,
            const std::pair<sm::vec3, sm::vec3>& e1) const {
            return e0.first < e1.first ||
                  (e0.first == e1.first && e0.second < e1.second);
        }
    }; // EdgeCmp
    std::map<std::pair<sm::vec3, sm::vec3>, Edge*, EdgeCmp> map2edge;
	for (auto& face_pos : faces_pos)
	{
        if (face_pos.size() < 2) {
            continue;
        }

		auto face = new Face();

		assert(face_pos.size() > 2);
		Edge* first = nullptr;
		Edge* last  = nullptr;

		for (int i = 0, n = face_pos.size(); i < n; ++i)
		{
            auto& pos = face_pos[i];
            auto& last_pos = face_pos[(i - 1 + n) % n];

			auto itr = map2vert.find(pos);
			assert(itr != map2vert.end());
			auto vert = itr->second;
			assert(vert);
			auto edge = new Edge(vert, face);
			if (!first) {
				first = edge;
			} else {
                assert(last);
                last->Connect(edge);
			}
			last = edge;

            map2edge.insert({ { last_pos, pos }, edge });
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

void Polyhedron::UpdateAABB()
{
	m_aabb.MakeEmpty();

    auto head = m_vertices.Head();
    auto v = head;
    do {
        m_aabb.Combine(v->position);
        v = v->linked_next;
    } while (v != head);
}

}