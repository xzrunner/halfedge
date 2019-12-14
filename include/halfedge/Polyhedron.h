#pragma once

#include "halfedge/DoublyLinkedList.h"
#include "halfedge/HalfEdge.h"
#include "halfedge/typedef.h"
#include "halfedge/TopoID.h"

#include <SM_Cube.h>
#include <SM_Plane.h>

#include <vector>
#include <memory>
#include <tuple>
#include <map>

namespace he
{

class Polyhedron
{
public:
    using in_vert           = std::pair<TopoID, sm::vec3>;
    using in_loop           = std::vector<size_t>;
    using in_face_no_hole   = std::pair<TopoID, in_loop>;
    using in_face_with_hole = std::tuple<TopoID, in_loop, std::vector<in_loop>>;

public:
    Polyhedron() {}
    Polyhedron(const Polyhedron& poly);
	Polyhedron(const sm::cube& aabb);
    Polyhedron(const std::vector<in_vert>& verts, const std::vector<in_face_no_hole>& faces); // right-hand
    Polyhedron(const std::vector<in_vert>& verts, const std::vector<in_face_with_hole>& faces); // right-hand
    Polyhedron& operator = (const Polyhedron& poly);

	auto& GetVerts() const { return m_verts; }
    auto& GetEdges() const { return m_edges; }
	auto& GetLoops() const { return m_loops; }

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

    void UniquePoints();

    enum ExtrudeFaceType
    {
        ExtrudeFront = 0,
        ExtrudeBack,
        ExtrudeSide,
        ExtrudeMaxCount,
    };
    bool Extrude(float distance, const std::vector<TopoID>& face_ids,
        bool create_face[ExtrudeMaxCount], std::vector<loop3*>* new_faces = nullptr);

    // test

    bool IsContain(const sm::vec3& pos) const;

    class EdgeCmp
    {
    public:
        bool operator () (
            const std::pair<size_t, size_t>& e0,
            const std::pair<size_t, size_t>& e1) const {
            return e0.first < e1.first ||
                (e0.first == e1.first && e0.second < e1.second);
        }
    }; // EdgeCmp

private:
    void OffsetTopoID(size_t v_off, size_t e_off, size_t f_off);

    void Clear();

    void BuildFromCube(const sm::cube& aabb);
    void BuildFromFaces(const std::vector<in_vert>& verts,
        const std::vector<in_face_no_hole>& faces);
    void BuildFromFaces(const std::vector<in_vert>& verts,
        const std::vector<in_face_with_hole>& faces);

    void BuildVertices(const std::vector<in_vert>& verts, std::vector<vert3*>& v_array);
    loop3* BuildLoop(TopoID id, const std::vector<size_t>& loop, const std::vector<vert3*>& v_array,
        std::map<std::pair<size_t, size_t>, edge3*, EdgeCmp>& map2edge);

    void RemoveFace(loop3* face);

private:
    DoublyLinkedList<vert3> m_verts;
    DoublyLinkedList<edge3> m_edges;
    DoublyLinkedList<loop3> m_loops;

    static size_t m_next_vert_id;
    static size_t m_next_edge_id;
    static size_t m_next_loop_id;

	sm::cube m_aabb;

}; // Polyhedron

}