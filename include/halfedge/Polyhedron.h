#pragma once

#include "halfedge/DoublyLinkedList.h"
#include "halfedge/HalfEdge.h"
#include "halfedge/typedef.h"
#include "halfedge/TopoID.h"

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
    Polyhedron(const std::vector<std::pair<TopoID, sm::vec3>>& vertices,
        const std::vector<std::pair<TopoID, std::vector<size_t>>>& faces); // right-hand
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
    std::vector<PolyhedronPtr> Subtract(const Polyhedron& subtrahend) const;

    // edit
    void Fill();
    void Fuse(float distance = 0.001f);
    static PolyhedronPtr Fuse(const std::vector<PolyhedronPtr>& polys, float distance = 0.001f);
    bool Extrude(float distance, const std::vector<TopoID>& face_ids,
        bool add_front, bool add_back, bool add_side);

    // test
    bool IsContain(const sm::vec3& pos) const;

private:
    void OffsetTopoID(size_t v_off, size_t e_off, size_t f_off);

    void Clear();

    void BuildFromCube(const sm::cube& aabb);
    void BuildFromPolygons(const std::vector<std::pair<TopoID, sm::vec3>>& vertices,
        const std::vector<std::pair<TopoID, std::vector<size_t>>>& faces);

    void RemoveFace(Face* face);

private:
    DoublyLinkedList<Vertex> m_vertices;
    DoublyLinkedList<Edge>   m_edges;
    DoublyLinkedList<Face>   m_faces;

    static size_t m_next_vert_id;
    static size_t m_next_edge_id;
    static size_t m_next_face_id;

	sm::cube m_aabb;

}; // Polyhedron

}