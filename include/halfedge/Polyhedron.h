#pragma once

#include "halfedge/HalfEdge.h"

#include <SM_Cube.h>

#include <vector>

namespace he
{

class Polyhedron
{
public:
	Polyhedron(const sm::cube& aabb);
	Polyhedron(const std::vector<std::vector<sm::vec3>>& faces);

	auto& GetVertices() const { return m_vertices; }
	auto& GetFaces() const { return m_faces; }

	const sm::cube& GetAABB() const { return m_aabb; }
	void UpdateAABB();

	static const float VERTEX_SCALE;

private:
	std::vector<VertexPtr> m_vertices;
	std::vector<FacePtr>   m_faces;

	sm::cube m_aabb;

}; // Polyhedron

using PolyhedronPtr = std::shared_ptr<Polyhedron>;

}