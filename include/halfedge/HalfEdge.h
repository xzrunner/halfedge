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
struct Loop;

template<typename T>
struct Edge
{
	Edge(Vertex<T>* vert, Loop<T>* loop, const TopoID& ids)
		: ids(ids)
        , vert(vert)
        , loop(loop)
    {
        vert->edge = this;
    }

    Edge<T>* Connect(Edge<T>* next);

    TopoID ids;

    Vertex<T>* vert = nullptr;     // vertex at the begin of the half-edge
    Loop<T>*   loop = nullptr;     // loop the half-edge borders

    Edge<T>*   twin = nullptr;     // oppositely oriented adjacent half-edge

    Edge<T>*   prev = nullptr;     // prev half-edge around the loop
    Edge<T>*   next = nullptr;     // next half-edge around the loop

    // double linked list
    Edge<T>* linked_prev = nullptr;
    Edge<T>* linked_next = nullptr;

}; // Edge

template<typename T>
struct Loop
{
    Loop(const TopoID& ids)
        : ids(ids)
    {
    }

    TopoID ids;

    // one of the half-edges bordering the loop
	Edge<T>* edge = nullptr;

    // double linked list
    Loop<T>* linked_prev = nullptr;
    Loop<T>* linked_next = nullptr;

}; // Loop

// edge
template<typename T>
void edge_make_pair(Edge<T>* e0, Edge<T>* e1);
template<typename T>
void edge_del_pair(Edge<T>* e);

template<typename T>
void bind_edge_loop(Loop<T>* loop, Edge<T>* edge);

typedef Vertex<sm::vec2> vert2;
typedef Edge<sm::vec2>   edge2;
typedef Loop<sm::vec2>   loop2;

typedef Vertex<sm::vec3> vert3;
typedef Edge<sm::vec3>   edge3;
typedef Loop<sm::vec3>   loop3;

}

#include "halfedge/HalfEdge.inl"