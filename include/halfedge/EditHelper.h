#pragma once

#include "halfedge/DoublyLinkedList.h"
#include "halfedge/HalfEdge.h"

namespace he
{

class EditHelper
{
public:
    static void UniquePoints(DoublyLinkedList<Vertex>& vts,
        const DoublyLinkedList<Edge>& edges, size_t& next_vert_id);

}; // EditHelper

}