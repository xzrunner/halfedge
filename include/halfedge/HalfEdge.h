#pragma once

#include <SM_Vector.h>
#include <SM_Plane.h>

#include <vector>

namespace he
{

struct Vertex;
struct Edge;
struct Face;

struct Vertex
{
	Vertex(sm::vec3 position, int id)
		: id(id)
        , position(position)
	{}

    int id = -1;

	sm::vec3 position;

    // one of the half-edges emantating from the vertex
    Edge* edge = nullptr;

    // double linked list
    Vertex* linked_prev = nullptr;
    Vertex* linked_next = nullptr;

}; // Vertex

struct Edge
{
	Edge(Vertex* vert, Face* face, int id)
		: id(id)
        , vert(vert)
        , face(face)
    {
        vert->edge = this;
    }

    Edge* Connect(Edge* next);

    int id = -1;

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
    Face(int id)
        : id(id)
    {
    }

    int id = -1;

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