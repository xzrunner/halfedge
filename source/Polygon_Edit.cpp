#include "halfedge/Polygon.h"
#include "halfedge/Utility.h"

#include <SM_Test.h>
#include <SM_Calc.h>

#include <iterator>

namespace
{

he::edge2* create_loop(const std::vector<sm::vec2>& new_pos, he::loop2* loop,
                       he::DoublyLinkedList<he::vert2>& verts,
                       he::DoublyLinkedList<he::edge2>& edges,
                       size_t& next_vert_id, size_t& next_edge_id)
{
    he::edge2* ret = nullptr;

    he::edge2* prev_edge = nullptr;
    for (size_t i = 0, n = new_pos.size(); i < n; ++i)
    {
        auto vert = new he::vert2(new_pos[i], next_vert_id++);
        auto edge = new he::edge2(vert, loop, next_edge_id++);
        if (prev_edge) {
            prev_edge->Connect(edge);
        } else {
            ret = edge;
        }
        prev_edge = edge;
    }

    prev_edge->Connect(ret);

    return ret;
}

void seam_loops(he::edge2* inner, he::edge2* outer, he::loop2* loop,
                he::DoublyLinkedList<he::edge2>& edges, size_t& next_edge_id)
{
    auto inner_next = inner->next;
    auto outer_prev = outer->prev;

    auto seam_in2out = new he::edge2(inner->vert, loop, next_edge_id++);
    inner->prev->Connect(seam_in2out)->Connect(outer);
    auto seam_out2in = new he::edge2(outer->vert, loop, next_edge_id++);
    outer_prev->Connect(seam_out2in)->Connect(inner);

    he::edge_make_pair(seam_in2out, seam_out2in);
}

void set_loop_pos(he::edge2* loop, const std::vector<sm::vec2>& verts)
{
    size_t ptr = 0;
    auto first_e = loop;
    auto curr_e = first_e;
    do {
        assert(ptr < verts.size());
        curr_e->vert->position = verts[ptr++];

        curr_e = curr_e->next;
    } while (curr_e != first_e);
}

void set_loop_vertices(he::edge2* dst, const he::edge2* src)
{
    size_t ptr = 0;
    auto first_e = dst;
    auto curr_e = first_e;
    auto src_e = src;
    do {
        curr_e->vert = src_e->vert;

        src_e = src_e->next;
        curr_e = curr_e->next;
    } while (curr_e != first_e);
}

std::vector<sm::vec2> calc_offset_loop(he::edge2* loop, float distance)
{
    std::vector<sm::vec2> new_loop;

    auto first_e = loop;
    auto curr_e = first_e;
    do {
        auto curr = curr_e->vert;
        auto prev = curr_e->prev->vert;
        auto next = curr_e->next->vert;
        auto angle = sm::get_angle(curr->position, prev->position, next->position);
        auto norm = -sm::rotate_vector(prev->position - curr->position, -angle/2).Normalized();
        auto new_p = curr->position + norm * distance / cos(angle / 2);
        new_loop.push_back(new_p);

        curr_e = curr_e->next;
    } while (curr_e != first_e);

    return new_loop;
}

void offset_loop(he::edge2* loop, float distance)
{
    auto new_loop = calc_offset_loop(loop, distance);

    size_t ptr = 0;
    auto first_e = loop;
    auto curr_e = first_e;
    do {
        curr_e->vert->position = new_loop[ptr++];
        curr_e = curr_e->next;
    } while (curr_e != first_e);
}

}

namespace he
{

//// todo: Edge event for distance < 0
bool Polygon::Offset(float distance, KeepType keep)
{
    if (distance == 0) {
        return false;
    }

    auto append_all = [&](loop2* loop, bool vert)
    {
        auto first_e = loop->edge;
        auto curr_e = first_e;
        do {
            if (vert) {
                m_verts.Append(curr_e->vert);
            }
            m_edges.Append(curr_e);

            curr_e = curr_e->next;
        } while (curr_e != first_e);

        m_loops.Append(loop);
    };

    auto face_to_border = [&](Face& face, float distance)
    {
        if (distance > 0)
        {
            auto hole = new loop2(m_next_loop_id++);
            auto new_loop = calc_offset_loop(face.border->edge, 0);
            hole->edge = create_loop(new_loop, hole, m_verts, m_edges, m_next_vert_id, m_next_edge_id);
            Utility::FlipLoop(*hole->edge);
            append_all(hole, true);

            face.holes.push_back(hole);

            offset_loop(face.border->edge, distance);
        }
        else
        {
            assert(distance < 0);

            auto hole = new loop2(m_next_loop_id++);
            auto new_loop = calc_offset_loop(face.border->edge, distance);
            hole->edge = create_loop(new_loop, hole, m_verts, m_edges, m_next_vert_id, m_next_edge_id);
            Utility::FlipLoop(*hole->edge);
            append_all(hole, true);

            face.holes.push_back(hole);
        }
    };

    switch (keep)
    {
    case KeepType::KeepInside:
    {
        if (distance > 0)
        {
            ;
        }
        else
        {
            assert(distance < 0);
            for (auto& face : m_faces)
            {
                offset_loop(face.border->edge, distance);
                for (auto& hole : face.holes) {
                    offset_loop(hole->edge, distance);
                }
            }
        }
    }
        break;
    case KeepType::KeepBorder:
        for (auto& face : m_faces) {
            face_to_border(face, distance);
        }
        break;
    case KeepType::KeepAll:
    {
        std::vector<Face> insides;
        for (auto& face : m_faces)
        {
            face_to_border(face, distance);

            auto inside = new loop2(m_next_loop_id++);
            inside->edge = Utility::CloneLoop(face.holes.front(), inside, m_next_edge_id);
            Utility::FlipLoop(*inside->edge);
            append_all(inside, false);
            insides.emplace_back(inside);
        }
        std::copy(insides.begin(), insides.end(), std::back_inserter(m_faces));
    }
        break;
    default:
        assert(0);
    }

    return true;
}

}