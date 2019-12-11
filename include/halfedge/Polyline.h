#pragma once

#include "halfedge/DoublyLinkedList.h"
#include "halfedge/HalfEdge.h"
#include "halfedge/typedef.h"
#include "halfedge/TopoID.h"

#include <SM_Vector.h>

#include <boost/noncopyable.hpp>

namespace he
{

class Polyline : boost::noncopyable
{
public:
    Polyline() {}
    Polyline(const Polyline& poly);
    Polyline(const std::vector<std::pair<TopoID, sm::vec3>>& vertices,
        const std::vector<std::pair<TopoID, std::vector<size_t>>>& polylines);
    Polyline& operator = (const Polyline& poly);

	auto& GetVertices() const  { return m_vertices; }
    auto& GetEdges() const     { return m_edges; }
    auto& GetPolylines() const { return m_polylines; }

    void Fuse(float distance = 0.001f);

    void UniquePoints();

private:
    void OffsetTopoID(size_t v_off, size_t e_off, size_t f_off);

    void Clear();

    void BuildFromPolylines(const std::vector<std::pair<TopoID, sm::vec3>>& vertices,
        const std::vector<std::pair<TopoID, std::vector<size_t>>>& polylines);

private:
    DoublyLinkedList<vert3> m_vertices;
    DoublyLinkedList<edge3>   m_edges;
    DoublyLinkedList<face3>   m_polylines;

    static size_t m_next_vert_id;
    static size_t m_next_edge_id;
    static size_t m_next_polyline_id;

}; // Polyline

}