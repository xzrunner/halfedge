#include "halfedge/Utility.h"

#include <SM_Calc.h>

#include <set>

namespace
{

std::vector<sm::vec2> dump_vertices(const he::loop2& loop)
{
    std::vector<sm::vec2> verts;

    auto first_e = loop.edge;
    auto curr_e = first_e;
    do {
        verts.push_back(curr_e->vert->position);
        curr_e = curr_e->next;
    } while (curr_e != first_e);

    return verts;
}

}

namespace he
{

bool Utility::IsLoopConvex(const loop2& loop)
{
    return sm::is_polygon_convex(dump_vertices(loop));
}

bool Utility::IsLoopClockwise(const loop2& loop)
{
    return sm::is_polygon_clockwise(dump_vertices(loop));
}

void Utility::LoopToVertices(const loop3& loop, std::vector<sm::vec3>& border)
{
    auto first_edge = loop.edge;
    auto curr_edge = first_edge;
    do {
        border.push_back(curr_edge->vert->position);
        curr_edge = curr_edge->next;
    } while (curr_edge != first_edge);
}

void Utility::LoopToPlane(const loop3& loop, sm::Plane& plane)
{
    auto curr_edge = loop.edge;
    auto first_edge = curr_edge;
    do {
        auto& v0 = curr_edge->vert->position;
        auto& v1 = curr_edge->next->vert->position;
        auto& v2 = curr_edge->next->next->vert->position;

        auto angle = sm::get_angle(v1, v0, v2);
        if (angle > std::numeric_limits<float>::epsilon() &&
            angle < SM_PI - std::numeric_limits<float>::epsilon()) {
            plane.Build(v0, v1, v2);
            plane.Flip();
            return;
        }

        curr_edge = curr_edge->next;
    } while (curr_edge != first_edge);

    assert(0);
}

}