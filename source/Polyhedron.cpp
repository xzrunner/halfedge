#include "halfedge/Polyhedron.h"

#include <SM_Vector.h>

#include <map>

namespace he
{

const float Polyhedron::VERTEX_SCALE = 0.01f;

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

	auto v1 = std::make_shared<Vertex>(p1);
	auto v2 = std::make_shared<Vertex>(p2);
	auto v3 = std::make_shared<Vertex>(p3);
	auto v4 = std::make_shared<Vertex>(p4);
	auto v5 = std::make_shared<Vertex>(p5);
	auto v6 = std::make_shared<Vertex>(p6);
	auto v7 = std::make_shared<Vertex>(p7);
	auto v8 = std::make_shared<Vertex>(p8);

	std::vector<VertexPtr> vertices;
	vertices.reserve(8);
	vertices.push_back(v1);
	vertices.push_back(v2);
	vertices.push_back(v3);
	vertices.push_back(v4);
	vertices.push_back(v5);
	vertices.push_back(v6);
	vertices.push_back(v7);
	vertices.push_back(v8);

	m_faces.reserve(6);

	// Front face
	auto f1 = std::make_shared<Face>();
	auto f1h1 = std::make_shared<Edge>(v1, f1);
	auto f1h2 = std::make_shared<Edge>(v5, f1);
	auto f1h3 = std::make_shared<Edge>(v6, f1);
	auto f1h4 = std::make_shared<Edge>(v2, f1);
	ConnectEdge(f1h1, f1h2);
	ConnectEdge(f1h2, f1h3);
	ConnectEdge(f1h3, f1h4);
	ConnectEdge(f1h4, f1h1);
	f1->start_edge = f1h1;
	m_faces.push_back(f1);

	// Left face
	auto f2 = std::make_shared<Face>();
	auto f2h1 = std::make_shared<Edge>(v1, f2);
	auto f2h2 = std::make_shared<Edge>(v2, f2);
	auto f2h3 = std::make_shared<Edge>(v4, f2);
	auto f2h4 = std::make_shared<Edge>(v3, f2);
	ConnectEdge(f2h1, f2h2);
	ConnectEdge(f2h2, f2h3);
	ConnectEdge(f2h3, f2h4);
	ConnectEdge(f2h4, f2h1);
	f2->start_edge = f2h1;
	m_faces.push_back(f2);

	// Bottom face
	auto f3 = std::make_shared<Face>();
	auto f3h1 = std::make_shared<Edge>(v1, f3);
	auto f3h2 = std::make_shared<Edge>(v3, f3);
	auto f3h3 = std::make_shared<Edge>(v7, f3);
	auto f3h4 = std::make_shared<Edge>(v5, f3);
	ConnectEdge(f3h1, f3h2);
	ConnectEdge(f3h2, f3h3);
	ConnectEdge(f3h3, f3h4);
	ConnectEdge(f3h4, f3h1);
	f3->start_edge = f3h1;
	m_faces.push_back(f3);

	// Top face
	auto f4 = std::make_shared<Face>();
	auto f4h1 = std::make_shared<Edge>(v2, f4);
	auto f4h2 = std::make_shared<Edge>(v6, f4);
	auto f4h3 = std::make_shared<Edge>(v8, f4);
	auto f4h4 = std::make_shared<Edge>(v4, f4);
	ConnectEdge(f4h1, f4h2);
	ConnectEdge(f4h2, f4h3);
	ConnectEdge(f4h3, f4h4);
	ConnectEdge(f4h4, f4h1);
	f4->start_edge = f4h1;
	m_faces.push_back(f4);

	// Back face
	auto f5 = std::make_shared<Face>();
	auto f5h1 = std::make_shared<Edge>(v3, f5);
	auto f5h2 = std::make_shared<Edge>(v4, f5);
	auto f5h3 = std::make_shared<Edge>(v8, f5);
	auto f5h4 = std::make_shared<Edge>(v7, f5);
	ConnectEdge(f5h1, f5h2);
	ConnectEdge(f5h2, f5h3);
	ConnectEdge(f5h3, f5h4);
	ConnectEdge(f5h4, f5h1);
	f5->start_edge = f5h1;
	m_faces.push_back(f5);

	// Right face
	auto f6 = std::make_shared<Face>();
	auto f6h1 = std::make_shared<Edge>(v5, f6);
	auto f6h2 = std::make_shared<Edge>(v7, f6);
	auto f6h3 = std::make_shared<Edge>(v8, f6);
	auto f6h4 = std::make_shared<Edge>(v6, f6);
	ConnectEdge(f6h1, f6h2);
	ConnectEdge(f6h2, f6h3);
	ConnectEdge(f6h3, f6h4);
	ConnectEdge(f6h4, f6h1);
	f6->start_edge = f6h1;
	m_faces.push_back(f6);

	// todo: Edge's twin
}

Polyhedron::Polyhedron(const std::vector<std::vector<sm::vec3>>& faces_pos)
{
	std::map<sm::vec3, he::VertexPtr, sm::Vector3Cmp> map2vert;
	for (auto& face : faces_pos) {
		for (auto& pos : face) {
			auto scaled = pos * VERTEX_SCALE;
			m_aabb.Combine(scaled);
			map2vert.insert({ scaled, std::make_shared<he::Vertex>(scaled) });
		}
	}
	m_vertices.reserve(map2vert.size());
	for (auto& itr : map2vert) {
		m_vertices.push_back(itr.second);
	}

	m_faces.reserve(faces_pos.size());
	for (auto& face_pos : faces_pos)
	{
		auto face = std::make_shared<Face>();

		assert(face_pos.size() > 2);
		EdgePtr first = nullptr;
		EdgePtr last = nullptr;
		for (auto& pos : face_pos)
		{
			auto scaled = pos * VERTEX_SCALE;
			auto itr = map2vert.find(scaled);
			assert(itr != map2vert.end());
			auto vert = itr->second;
			assert(vert);
			auto edge = std::make_shared<Edge>(vert, face);
			if (!first) {
				first = edge;
			} else {
				ConnectEdge(last, edge);
			}
			last = edge;
		}
		ConnectEdge(last, first);

		face->start_edge = first;

		m_faces.push_back(face);
	}

	// twin half edge
	for (auto& vert : map2vert)
	{
		for (auto& edge : vert.second->output)
		{
			if (edge->twin) {
				continue;
			}

			auto twin = nullptr;
			for (auto& out_edge : edge->next->origin->output)
			{
				if (out_edge->next->origin.get() == vert.second.get())
				{
					edge->twin = out_edge;
					assert(!out_edge->twin);
					out_edge->twin = edge;
					break;
				}
			}
		}
	}

	//// check
	//for (auto& vert : map2vert)
	//{
	//	for (auto& edge : vert.second->output)
	//	{
	//		assert(edge->twin && edge->twin->twin == edge);
	//		assert(edge->next->origin == edge->twin->origin);
	//		assert(edge->origin == edge->twin->next->origin);
	//	}
	//	for (auto& edge : vert.second->input)
	//	{
	//		assert(edge->twin && edge->twin->twin == edge);
	//		assert(edge->next->origin == edge->twin->origin);
	//		assert(edge->origin == edge->twin->next->origin);
	//	}
	//}
}

}