#include "halfedge/Polygon.h"

#include <SM_Test.h>
#include <SM_Calc.h>

namespace
{

he::edge2* create_loop(const std::vector<sm::vec2>& verts, he::face2* face,
                       he::DoublyLinkedList<he::vert2>& vertices,
                       he::DoublyLinkedList<he::edge2>& edges,
                       size_t& next_vert_id, size_t& next_edge_id)
{
    he::edge2* ret = nullptr;

    he::edge2* prev_edge = nullptr;
    for (size_t i = 0, n = verts.size(); i < n; ++i)
    {
        auto vert = new he::vert2(verts[i], next_vert_id++);
        auto edge = new he::edge2(vert, face, next_edge_id++);
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

he::edge2* clone_loop(const he::face2* old_face, he::face2* new_face,
                      he::DoublyLinkedList<he::vert2>& vertices, he::DoublyLinkedList<he::edge2>& edges,
                      size_t& next_vert_id, size_t& next_edge_id)
{
    he::edge2* ret = nullptr;

    he::edge2* prev_edge = nullptr;
    auto first_e = old_face->edge;
    auto curr_e = first_e;
    do {
        auto edge = new he::edge2(curr_e->vert, new_face, next_edge_id++);
        if (prev_edge) {
            prev_edge->Connect(edge);
        } else {
            ret = edge;
        }
        prev_edge = edge;

        curr_e = curr_e->next;
    } while (curr_e != first_e);

    prev_edge->Connect(ret);

    return ret;
}

void flip_loop(he::edge2* loop)
{
    std::vector<he::edge2*> edges;

    auto first_e = loop;
    auto curr_e = first_e;
    do {
        edges.push_back(curr_e);
        curr_e = curr_e->next;
    } while (curr_e != first_e);

    for (size_t i = 0, n = edges.size(); i < n; ++i) {
        edges[i]->Connect(edges[(i + n - 1) % n]);
    }
}

void seam_loops(he::edge2* inner, he::edge2* outer, he::face2* face,
                he::DoublyLinkedList<he::edge2>& edges, size_t& next_edge_id)
{
    auto inner_next = inner->next;
    auto outer_prev = outer->prev;

    auto seam_in2out = new he::edge2(inner->vert, face, next_edge_id++);
    inner->prev->Connect(seam_in2out)->Connect(outer);
    auto seam_out2in = new he::edge2(outer->vert, face, next_edge_id++);
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

}

namespace he
{

// todo: Edge event for distance < 0
bool Polygon::Offset(float distance, KeepType keep)
{
    if (distance == 0) {
        return false;
    }

    if (!IsConvex()) {
        return false;
    }

    if (distance > 0 && keep == KeepType::KeepInside) {
        return true;
    }

    // calc new border
    std::vector<std::vector<sm::vec2>> new_loops;
    auto first_f = m_faces.Head();
    auto curr_f = first_f;
    do {
        std::vector<sm::vec2> new_loop;

        auto first_e = curr_f->edge;
        auto curr_e = first_e;
        do {
            auto curr = curr_e->vert;
            auto prev = curr_e->prev->vert;
            auto next = curr_e->next->vert;
            auto angle = sm::get_angle(curr->position, prev->position, next->position);
            auto norm = -sm::rotate_vector(prev->position - curr->position, -angle/2).Normalized();
            auto new_p = curr->position + norm * distance;
            new_loop.push_back(new_p);

            curr_e = curr_e->next;
        } while (curr_e != first_e);

        new_loops.push_back(new_loop);

        curr_f = curr_f->linked_next;
    } while (curr_f != first_f);

    if (distance < 0 && keep == KeepType::KeepInside)
    {
        size_t ptr = 0;
        auto first_f = m_faces.Head();
        auto curr_f = first_f;
        do {
            set_loop_pos(curr_f->edge, new_loops[ptr++]);
            curr_f = curr_f->linked_next;
        } while (curr_f != first_f);

        return true;
    }

    if (distance > 0)
    {
        if (keep == KeepType::KeepBorder)
        {
            size_t ptr_face = 0;
            auto first_f = m_faces.Head();
            auto curr_f = first_f;
            do {
                // inner
                auto new_in = curr_f->edge;
                flip_loop(new_in);
                // outer
                auto new_out = create_loop(new_loops[ptr_face], curr_f, m_verts, m_edges, m_next_vert_id, m_next_edge_id);
                // seam
                seam_loops(new_in, new_out, curr_f, m_edges, m_next_edge_id);

                ++ptr_face;
                curr_f = curr_f->linked_next;
            } while (curr_f != first_f);
        }
        else
        {
            assert(keep == KeepType::KeepAll);

            size_t ptr_face = 0;
            std::vector<face2*> new_faces;
            auto first_f = m_faces.Head();
            auto curr_f = first_f;
            do {
                auto new_f = new face2(m_next_face_id++);

                // inner
                auto new_in = clone_loop(curr_f, new_f, m_verts, m_edges, m_next_vert_id, m_next_edge_id);
                flip_loop(new_in);
                // outer
                auto new_out = create_loop(new_loops[ptr_face], new_f, m_verts, m_edges, m_next_vert_id, m_next_edge_id);
                // seam
                seam_loops(new_in, new_out, new_f, m_edges, m_next_edge_id);
                // pair
                auto first_e = curr_f->edge;
                auto curr_e = first_e;
                auto new_e = new_in->prev;
                do {
                    he::edge_make_pair(curr_e, new_e);
                    new_e = new_e->prev;
                    curr_e = curr_e->next;
                } while (curr_e != first_e);

                new_f->edge = new_in;
                new_faces.push_back(new_f);

                ++ptr_face;
                curr_f = curr_f->linked_next;
            } while (curr_f != first_f);

            for (auto& f : new_faces) {
                m_faces.Append(f);
            }
        }
    }
    else
    {
        if (keep == KeepType::KeepBorder)
        {
            size_t ptr_face = 0;
            auto first_f = m_faces.Head();
            auto curr_f = first_f;
            do {
                // inner
                auto new_in = create_loop(new_loops[ptr_face], curr_f, m_verts, m_edges, m_next_vert_id, m_next_edge_id);
                flip_loop(new_in);
                // outer
                auto new_out = curr_f->edge;
                // seam
                seam_loops(new_in, new_out, curr_f, m_edges, m_next_edge_id);

                ++ptr_face;
                curr_f = curr_f->linked_next;
            } while (curr_f != first_f);
        }
        else
        {
            assert(keep == KeepType::KeepAll);

            size_t ptr_face = 0;
            std::vector<face2*> new_faces;
            auto first_f = m_faces.Head();
            auto curr_f = first_f;
            do {
                auto new_f = new face2(m_next_face_id++);

                // outer
                auto new_out = clone_loop(curr_f, new_f, m_verts, m_edges, m_next_vert_id, m_next_edge_id);
                // inner
                auto new_in = create_loop(new_loops[ptr_face], new_f, m_verts, m_edges, m_next_vert_id, m_next_edge_id);
                // offset origin
                set_loop_vertices(curr_f->edge, new_in);
                flip_loop(new_in);
                // seam
                seam_loops(new_in, new_out, new_f, m_edges, m_next_edge_id);
                // pair
                auto first_e = curr_f->edge;
                auto curr_e = first_e;
                auto new_e = new_in->prev;
                do {
                    he::edge_make_pair(curr_e, new_e);
                    new_e = new_e->prev;
                    curr_e = curr_e->next;
                } while (curr_e != first_e);

                new_f->edge = new_in;
                new_faces.push_back(new_f);

                ++ptr_face;
                curr_f = curr_f->linked_next;
            } while (curr_f != first_f);

            for (auto& f : new_faces) {
                m_faces.Append(f);
            }
        }
    }

    return true;
}

bool Polygon::IsConvex() const
{
    auto first_f = m_faces.Head();
    auto curr_f = first_f;
    do {
        std::vector<sm::vec2> loop;

        auto first_e = curr_f->edge;
        auto curr_e = first_e;
        do {
            loop.push_back(curr_e->vert->position);
            curr_e = curr_e->next;
        } while (curr_e != first_e);

        if (!sm::is_polygon_convex(loop)) {
            return false;
        }

        curr_f = curr_f->linked_next;
    } while (curr_f != first_f);

    return true;
}

}