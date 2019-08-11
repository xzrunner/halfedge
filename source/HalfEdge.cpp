#include "halfedge/HalfEdge.h"

namespace he
{

Edge* Edge::Connect(Edge* next_edge)
{
    assert(!next && !next_edge->prev);
    next_edge->prev = this;
    next = next_edge;
    return next;
}

void edge_make_pair(Edge* e0, Edge* e1)
{
    assert(!e0->twin && !e1->twin);
    e0->twin = e1;
    e1->twin = e0;
}

void face_to_vertices(const Face& face, std::vector<sm::vec3>& border)
{
	auto ptr = face.edge;
	while (true)
	{
		border.push_back(ptr->vert->position);

		ptr = ptr->next;
		if (ptr == face.edge) {
			break;
		}
	}
}

void face_to_plane(const Face& face, sm::Plane& plane)
{
	auto& v0 = face.edge->vert->position;
	auto& v1 = face.edge->next->vert->position;
	auto& v2 = face.edge->next->next->vert->position;
	plane.Build(v0, v1, v2);
}

}