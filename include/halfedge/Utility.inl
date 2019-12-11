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

}