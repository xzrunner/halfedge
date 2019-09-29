#include "halfedge/Polyhedron.h"

#include <SM_Calc.h>

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
            auto ret = new_edges.insert({ edge->vert, edge });
            assert(ret.second);
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

void Polyhedron::Fuse(float distance)
{
    auto v0_first = m_vertices.Head();
    auto v0_curr = v0_first;
    do {
        auto v1_curr = v0_curr->linked_next;
        do {
            // find
            auto d = sm::dis_pos3_to_pos3(
                v0_curr->position, v1_curr->position
            );
            if (d < distance)
            {
                // change edges
                auto first_edge = v1_curr->edge;
                auto curr_edge = first_edge;
                do {
                    curr_edge->vert = v0_curr;
                    if (!curr_edge->twin) {
                        break;
                    }
                    curr_edge = curr_edge->twin->next;
                } while (curr_edge != first_edge);

                curr_edge = first_edge;
                do {
                    curr_edge->vert = v0_curr;
                    curr_edge = curr_edge->prev->twin;
                } while (curr_edge && curr_edge != first_edge);

                // rm vertex
                m_vertices.Remove(v1_curr);
            }

            v1_curr = v1_curr->linked_next;
        } while (v1_curr != v0_first && v1_curr != v0_curr);

        v0_curr = v0_curr->linked_next;
    } while (v0_curr != v0_first);
}

}