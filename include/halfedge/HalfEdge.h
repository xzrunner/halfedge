#pragma once

#include "halfedge/TopoID.h"

#include <SM_Vector.h>
#include <SM_Plane.h>

namespace he
{

struct Vertex;
struct Edge;
struct Face;

struct Vertex
{
	Vertex(sm::vec3 position, const TopoID& ids)
		: ids(ids)
        , position(position)
	{
    }

    TopoID ids;

	sm::vec3 position;

    // one of the half-edges emantating from the vertex
    Edge* edge = nullptr;

    // double linked list
    Vertex* linked_prev = nullptr;
    Vertex* linked_next = nullptr;

}; // Vertex

struct Edge
{
	Edge(Vertex* vert, Face* face, const TopoID& ids)
		: ids(ids)
        , vert(vert)
        , face(face)
    {
        vert->edge = this;
    }

    Edge* Connect(Edge* next);

    TopoID ids;

    Vertex* vert = nullptr;     // vertex at the begin of the half-edge
    Face*   face = nullptr;     // face the half-edge borders

    Edge*   twin = nullptr;     // oppositely oriented adjacent half-edge

    Edge*   prev = nullptr;     // prev half-edge around the face
    Edge*   next = nullptr;     // next half-edge around the face

    // double linked list
    Edge* linked_prev = nullptr;
    Edge* linked_next = nullptr;

}; // Edge

struct Face
{
    Face(const TopoID& ids)
        : ids(ids)
    {
    }

    TopoID ids;

    // one of the half-edges bordering the face
	Edge* edge = nullptr;

    // double linked list
    Face* linked_prev = nullptr;
    Face* linked_next = nullptr;

}; // Face

// edge
void edge_make_pair(Edge* e0, Edge* e1);

// face
void face_to_vertices(const Face& face, std::vector<sm::vec3>& border);
void face_to_plane(const Face& face, sm::Plane& plane);

void bind_edge_face(Face* face, Edge* edge);

}