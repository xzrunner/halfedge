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

    // face
    static void face_to_vertices(const face3& face, std::vector<sm::vec3>& border);
    static void face_to_plane(const face3& face, sm::Plane& plane);

}; // Utility

}

#include "halfedge/Utility.inl"