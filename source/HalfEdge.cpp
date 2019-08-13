#include "halfedge/HalfEdge.h"

#include <SM_Calc.h>

namespace he
{

Edge* Edge::Connect(Edge* next_edge)
{
    next_edge->prev = this;
    next = next_edge;
    return next;
}

void edge_make_pair(Edge* e0, Edge* e1)
{
    e0->twin = e1;
    e1->twin = e0;
}

void face_to_vertices(const Face& face, std::vector<sm::vec3>& border)
{
    auto first_edge = face.edge;
    auto curr_edge = first_edge;
    do {
        border.push_back(curr_edge->vert->position);
        curr_edge = curr_edge->next;
    } while (curr_edge != first_edge);
}

void face_to_plane(const Face& face, sm::Plane& plane)
{
    auto curr_edge = face.edge;
    auto first_edge = curr_edge;
    do {
        auto& v0 = curr_edge->vert->position;
        auto& v1 = curr_edge->next->vert->position;
        auto& v2 = curr_edge->next->next->vert->position;

        auto angle = sm::get_angle(v1, v0, v2);
        if (angle > std::numeric_limits<float>::epsilon() &&
            angle < SM_PI - std::numeric_limits<float>::epsilon()) {
            plane.Build(v0, v1, v2);
            plane.Flip();
            return;
        }

        curr_edge = curr_edge->next;
    } while (curr_edge != first_edge);

    assert(0);
}

void bind_edge_face(Face* face, Edge* edge)
{
    face->edge = edge;

    auto curr_edge = edge;
    do {
        curr_edge->face = face;
        curr_edge = curr_edge->next;
    } while (curr_edge != edge);
}

}