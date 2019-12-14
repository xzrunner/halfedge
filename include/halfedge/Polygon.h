#pragma once

#include "halfedge/DoublyLinkedList.h"
#include "halfedge/HalfEdge.h"

#include <boost/noncopyable.hpp>

#include <map>

namespace he
{

class Polygon : boost::noncopyable
{
public:
    Polygon() {}
    Polygon(const Polygon& poly);
    Polygon(const std::vector<std::pair<TopoID, sm::vec2>>& verts,
        const std::vector<std::pair<TopoID, std::vector<size_t>>>& borders,
        const std::vector<std::pair<TopoID, std::vector<size_t>>>& holes);
    Polygon& operator = (const Polygon& poly);

    auto& GetVerts() const { return m_verts; }
    auto& GetEdges() const    { return m_edges; }

    auto& GetBorders() const { return m_borders; }
    auto& GetHoles() const { return m_holes; }

    enum class KeepType
    {
        KeepInside,
        KeepBorder,
        KeepAll,
    };
    bool Offset(float distance, KeepType keep = KeepType::KeepAll);

private:
    void Clear();

    void BuildFromLoops(const std::vector<std::pair<TopoID, sm::vec2>>& verts,
        const std::vector<std::pair<TopoID, std::vector<size_t>>>& borders,
        const std::vector<std::pair<TopoID, std::vector<size_t>>>& holes);

    loop2* CreateLoop(const std::vector<vert2*>& verts, const std::pair<TopoID, std::vector<size_t>>& loop);

    static std::vector<std::pair<TopoID, sm::vec2>>
        DumpVertices(const DoublyLinkedList<vert2>& verts, std::map<vert2*, size_t>& vert2idx);
    static std::vector<std::pair<TopoID, std::vector<size_t>>>
        DumpLoops(const DoublyLinkedList<loop2>& loops, const std::map<vert2*, size_t>& vert2idx);

private:
    DoublyLinkedList<vert2> m_verts;
    DoublyLinkedList<edge2> m_edges;

    DoublyLinkedList<loop2> m_borders;
    DoublyLinkedList<loop2> m_holes;

    static size_t m_next_vert_id;
    static size_t m_next_edge_id;
    static size_t m_next_loop_id;

}; // Polygon

}