#pragma once

#include "halfedge/DoublyLinkedList.h"
#include "halfedge/HalfEdge.h"

#include <SM_Cube.h>

#include <vector>
#include <memory>

namespace he
{

class Polyhedron
{
public:
	Polyhedron(const sm::cube& aabb);
	Polyhedron(const std::vector<std::vector<sm::vec3>>& faces);

	auto& GetVertices() const { return m_vertices; }
    auto& GetEdges() const    { return m_edges; }
	auto& GetFaces() const    { return m_faces; }

	const sm::cube& GetAABB() const { return m_aabb; }
	void UpdateAABB();

private:
    DoublyLinkedList<Vertex> m_vertices;
    DoublyLinkedList<Edge>   m_edges;
    DoublyLinkedList<Face>   m_faces;

	sm::cube m_aabb;

}; // Polyhedron

using PolyhedronPtr = std::shared_ptr<Polyhedron>;

}