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
    Polyhedron(const std::vector<sm::vec3>& vertices,
        const std::vector<std::vector<size_t>>& faces); // right-hand
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

    // edit
    void Fill();
    void Fuse(float distance = 0.001f);

private:
    void Clear();

    void BuildFromCube(const sm::cube& aabb);
    void BuildFromPolygons(const std::vector<sm::vec3>& vertices,
        const std::vector<std::vector<size_t>>& faces);

private:
    DoublyLinkedList<Vertex> m_vertices;
    DoublyLinkedList<Edge>   m_edges;
    DoublyLinkedList<Face>   m_faces;

    int m_next_vert_id = 0;
    int m_next_edge_id = 0;
    int m_next_face_id = 0;

	sm::cube m_aabb;

}; // Polyhedron

}