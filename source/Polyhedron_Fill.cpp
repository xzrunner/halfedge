#include "halfedge/Polyhedron.h"

#include <map>

namespace he
{

void Polyhedron::Fill()
{
    std::map<Vertex*, Edge*> new_edges;

    // create edges
    auto first = m_edges.Head();
    auto curr = first;
    do {
        if (!curr->twin)
        {
            auto edge = new Edge(curr->next->vert, nullptr, m_next_edge_id++);
            new_edges.insert({ edge->vert, edge });
            edge_make_pair(edge, curr);
            m_edges.Append(edge);
        }
        curr = curr->linked_next;
    } while (curr != first);

    // create face and connect new edges
    for (auto itr = new_edges.begin(); itr != new_edges.end(); ++itr)
    {
        Edge* edge = itr->second;
        if (edge->face) {
            continue;
        }

        auto face = new Face(m_next_face_id++);
        face->edge = edge;

        Vertex* first = edge->vert;
        do {
            edge->face = face;

            auto itr = new_edges.find(edge->twin->vert);
            assert(itr != new_edges.end());
            auto next_edge = itr->second;

            edge->Connect(next_edge);
            edge = next_edge;
        } while (edge->vert != first);

        m_faces.Append(face);
    }
}

}