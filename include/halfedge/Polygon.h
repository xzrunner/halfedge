#pragma once

#include "halfedge/DoublyLinkedList.h"
#include "halfedge/HalfEdge.h"

#include <boost/noncopyable.hpp>

namespace he
{

class Polygon : boost::noncopyable
{
public:
    Polygon() {}
    Polygon(const Polygon& poly);
    Polygon(const std::vector<std::pair<TopoID, sm::vec2>>& vertices,
        const std::vector<std::pair<TopoID, std::vector<size_t>>>& faces);
    Polygon& operator = (const Polygon& poly);

    auto& GetVertices() const { return m_verts; }
    auto& GetEdges() const    { return m_edges; }
    auto& GetFaces() const    { return m_faces; }

    enum class KeepType
    {
        KeepInside,
        KeepBorder,
        KeepAll,
    };
    bool Offset(float distance, KeepType keep = KeepType::KeepAll);

    bool IsConvex() const;

private:
    void Clear();

    void BuildFromFaces(const std::vector<std::pair<TopoID, sm::vec2>>& vertices,
        const std::vector<std::pair<TopoID, std::vector<size_t>>>& faces);

private:
    DoublyLinkedList<vert2> m_verts;
    DoublyLinkedList<edge2> m_edges;
    DoublyLinkedList<face2> m_faces;

    static size_t m_next_vert_id;
    static size_t m_next_edge_id;
    static size_t m_next_face_id;

}; // Polygon

}