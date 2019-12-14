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

    // face3
    static void FaceToVertices(const face3& face, std::vector<sm::vec3>& border);
    static void FaceToPlane(const face3& face, sm::Plane& plane);

}; // Utility

}

#include "halfedge/Utility.inl"