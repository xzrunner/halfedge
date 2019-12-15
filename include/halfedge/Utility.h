#pragma once

#include "halfedge/DoublyLinkedList.h"
#include "halfedge/HalfEdge.h"

#include <SM_Plane.h>

namespace he
{

class Utility
{
public:
    template<typename T>
    static void UniquePoints(DoublyLinkedList<Vertex<T>>& vts,
        const DoublyLinkedList<Edge<T>>& edges, size_t& next_vert_id);

    template<typename T>
    static void FlipLoop(Loop<T>& loop);
    template<typename T>
    static void FlipLoop(Edge<T>& edge);

    template<typename T>
    static Edge<T>* CloneLoop(const Loop<T>* old_loop, Loop<T>* new_loop, size_t& next_edge_id);

    template<typename T>
    static size_t EdgeSize(const Loop<T>& loop);

    // loop2
    static bool IsLoopConvex(const loop2& loop);
    static bool IsLoopClockwise(const loop2& loop);

    // loop3
    static void LoopToVertices(const loop3& loop, std::vector<sm::vec3>& border);
    static void LoopToPlane(const loop3& loop, sm::Plane& plane);

}; // Utility

}

#include "halfedge/Utility.inl"