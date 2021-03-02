#pragma once

#include "halfedge/DoublyLinkedList.h"
#include "halfedge/HalfEdge.h"
#include "halfedge/noncopyable.h"

#include <map>

namespace he
{

class Polygon : noncopyable
{
public:
    struct Face
    {
        Face(loop2* border = nullptr, std::vector<loop2*>& holes = std::vector<loop2*>())
            : border(border)
            , holes(holes)
        {
        }

        loop2* border = nullptr;
        std::vector<loop2*> holes;
    };

    using in_vert = std::pair<TopoID, sm::vec2>;
    using in_loop = std::vector<size_t>;
    using in_face = std::tuple<TopoID, in_loop, std::vector<in_loop>>;

public:
    Polygon() {}
    Polygon(const Polygon& poly);
    Polygon(const std::vector<in_vert>& verts, const std::vector<in_face>& faces);
    Polygon& operator = (const Polygon& poly);

    auto& GetVerts() const { return m_verts; }
    auto& GetEdges() const { return m_edges; }
    auto& GetFaces() const { return m_faces; }

    enum class KeepType
    {
        KeepInside,
        KeepBorder,
        KeepAll,
    };
    bool Offset(float distance, KeepType keep = KeepType::KeepAll);

private:
    void Clear();

    void BuildFromFaces(const std::vector<in_vert>& verts, const std::vector<in_face>& faces);

    loop2* CreateLoop(const std::vector<vert2*>& verts, TopoID id, const std::vector<size_t>& loop);

    static std::vector<in_vert> DumpVertices(const DoublyLinkedList<vert2>& verts, std::map<vert2*, size_t>& vert2idx);
    static in_loop DumpLoop(const loop2& loop, const std::map<vert2*, size_t>& vert2idx);

private:
    DoublyLinkedList<vert2> m_verts;
    DoublyLinkedList<edge2> m_edges;
    DoublyLinkedList<loop2> m_loops;

    std::vector<Face> m_faces;

    static size_t m_next_vert_id;
    static size_t m_next_edge_id;
    static size_t m_next_loop_id;

}; // Polygon

}