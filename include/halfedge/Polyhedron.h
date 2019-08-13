#pragma once

#include "halfedge/DoublyLinkedList.h"
#include "halfedge/HalfEdge.h"
#include "halfedge/typedef.h"

#include <SM_Cube.h>

#include <vector>
#include <memory>

namespace he
{

class Polyhedron
{
public:
    Polyhedron() {}
    Polyhedron(const Polyhedron& poly);
	Polyhedron(const sm::cube& aabb);
	Polyhedron(const std::vector<std::vector<sm::vec3>>& polygons);
    Polyhedron& operator = (const Polyhedron& poly);

	auto& GetVertices() const { return m_vertices; }
    auto& GetEdges() const    { return m_edges; }
	auto& GetFaces() const    { return m_faces; }

	const sm::cube& GetAABB() const { return m_aabb; }
	void UpdateAABB();

    enum class KeepType
    {
        KeepAbove,
        KeepBelow,
        KeepAll,
    };
    bool Clip(const sm::Plane& plane, KeepType keep, bool seam_face = false);

    // boolean
    PolyhedronPtr Union(const Polyhedron& other) const;
    PolyhedronPtr Intersect(const Polyhedron& other) const;
    PolyhedronPtr Subtract(const Polyhedron& subtrahend) const;

private:
    void Clear();

    void BuildFromCube(const sm::cube& aabb);
    void BuildFromPolygons(const std::vector<std::vector<sm::vec3>>& polygons);

private:
    DoublyLinkedList<Vertex> m_vertices;
    DoublyLinkedList<Edge>   m_edges;
    DoublyLinkedList<Face>   m_faces;

	sm::cube m_aabb;

}; // Polyhedron

}