#pragma once

#include "halfedge/TopoID.h"

#include <SM_Vector.h>

namespace he
{

template<typename T>
struct Edge;

template<typename T>
struct Vertex
{
	Vertex(const T& position, const TopoID& ids)
		: ids(ids)
        , position(position)
	{
    }

    TopoID ids;

	T position;

    // one of the half-edges emantating from the vertex
    Edge<T>* edge = nullptr;

    // double linked list
    Vertex<T>* linked_prev = nullptr;
    Vertex<T>* linked_next = nullptr;

}; // Vertex

template<typename T>
struct Face;

template<typename T>
struct Edge
{
	Edge(Vertex<T>* vert, Face<T>* face, const TopoID& ids)
		: ids(ids)
        , vert(vert)
        , face(face)
    {
        vert->edge = this;
    }

    Edge<T>* Connect(Edge<T>* next);

    TopoID ids;

    Vertex<T>* vert = nullptr;     // vertex at the begin of the half-edge
    Face<T>*   face = nullptr;     // face the half-edge borders

    Edge<T>*   twin = nullptr;     // oppositely oriented adjacent half-edge

    Edge<T>*   prev = nullptr;     // prev half-edge around the face
    Edge<T>*   next = nullptr;     // next half-edge around the face

    // double linked list
    Edge<T>* linked_prev = nullptr;
    Edge<T>* linked_next = nullptr;

}; // Edge

template<typename T>
struct Face
{
    Face(const TopoID& ids)
        : ids(ids)
    {
    }

    TopoID ids;

    // one of the half-edges bordering the face
	Edge<T>* edge = nullptr;

    // double linked list
    Face<T>* linked_prev = nullptr;
    Face<T>* linked_next = nullptr;

}; // Face

// edge
template<typename T>
void edge_make_pair(Edge<T>* e0, Edge<T>* e1);

template<typename T>
void bind_edge_face(Face<T>* face, Edge<T>* edge);

typedef Vertex<sm::vec2> vert2;
typedef Edge<sm::vec2>   edge2;
typedef Face<sm::vec2>   face2;

typedef Vertex<sm::vec3> vert3;
typedef Edge<sm::vec3>   edge3;
typedef Face<sm::vec3>   face3;

}

#include "halfedge/HalfEdge.inl"