#pragma once

namespace he
{

template<typename T>
void Utility::UniquePoints(DoublyLinkedList<Vertex<T>>& vts,
                           const DoublyLinkedList<Edge<T>>& edges,
                           size_t& next_vert_id)
{
    std::set<Vertex<T>*> unique;

    auto first_edge = edges.Head();
    auto curr_edge = first_edge;
    do {
        auto v = curr_edge->vert;
        auto itr = unique.find(v);
        if (itr == unique.end())
        {
            unique.insert(v);
        }
        else
        {
            auto new_v = new Vertex<T>(v->position, v->ids);
            new_v->ids.Append(next_vert_id++);
            vts.Append(new_v);
            curr_edge->vert = new_v;
        }

        curr_edge = curr_edge->linked_next;
    } while (curr_edge != first_edge);
}

template<typename T>
void Utility::FlipLoop(Loop<T>& loop)
{
    if (loop.edge) {
        FlipLoop(*loop.edge);
    }
}

template<typename T>
void Utility::FlipLoop(Edge<T>& edge)
{
    std::vector<Edge<T>*> edges;

    auto first_e = &edge;
    auto curr_e = first_e;
    do {
        edges.push_back(curr_e);
        curr_e = curr_e->next;
    } while (curr_e != first_e);

    for (size_t i = 0, n = edges.size(); i < n; ++i) {
        edges[i]->Connect(edges[(i + n - 1) % n]);
    }
}

template<typename T>
Edge<T>* Utility::CloneLoop(const Loop<T>* old_loop, Loop<T>* new_loop, size_t& next_edge_id)
{
    Edge<T>* ret = nullptr;

    Edge<T>* prev_edge = nullptr;
    auto first_e = old_loop->edge;
    auto curr_e = first_e;
    do {
        auto edge = new Edge<T>(curr_e->vert, new_loop, next_edge_id++);
        if (prev_edge) {
            prev_edge->Connect(edge);
        } else {
            assert(!ret);
            ret = edge;
        }
        prev_edge = edge;

        curr_e = curr_e->next;
    } while (curr_e != first_e);

    prev_edge->Connect(ret);

    return ret;
}

template<typename T>
size_t Utility::EdgeSize(const Loop<T>& loop)
{
    size_t num = 0;

    auto first_e = loop.edge;
    auto curr_e = first_e;
    do {
        ++num;

        curr_e = curr_e->next;
    } while (curr_e != first_e);

    return num;
}

}