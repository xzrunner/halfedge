#pragma once

namespace he
{

template<typename T>
Edge<T>* Edge<T>::Connect(Edge<T>* next_edge)
{
    if (next_edge) {
        next_edge->prev = this;
    }
    next = next_edge;
    return next;
}

template<typename T>
void edge_make_pair(Edge<T>* e0, Edge<T>* e1)
{
    assert(!e0->twin || e0->twin == e1);
    assert(!e1->twin || e1->twin == e0);

    e0->twin = e1;
    e1->twin = e0;
}

template<typename T>
void edge_del_pair(Edge<T>* e)
{
    if (e->twin) {
        e->twin->twin = nullptr;
        e->twin = nullptr;
    }
}

template<typename T>
void bind_edge_loop(Loop<T>* loop, Edge<T>* edge)
{
    loop->edge = edge;

    auto curr_edge = edge;
    do {
        curr_edge->loop = loop;
        curr_edge = curr_edge->next;
    } while (curr_edge != edge);
}

}