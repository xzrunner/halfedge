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

const float POINT_STATUS_EPSILON = 0.0001f;

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
    sm::vec3 normal;

    auto curr_edge = loop.edge;
    auto first_edge = curr_edge;
    do {
        auto& v0 = curr_edge->vert->position;
        auto& v1 = curr_edge->next->vert->position;

        normal += v0.Cross(v1);

        curr_edge = curr_edge->next;
    } while (curr_edge != first_edge);

    plane.Build(normal, first_edge->vert->position);
    plane.Flip();
}

sm::vec3 Utility::CalcLoopNorm(const loop3& loop)
{
    sm::Plane plane;
    LoopToPlane(loop, plane);
    return plane.normal;
}

sm::vec3 Utility::CalcFaceNorm(const Polyhedron::Face& face)
{
    if (face.border) {
        return CalcLoopNorm(*face.border);
    } else {
        for (auto& hole : face.holes) {
            return -CalcLoopNorm(*hole);
        }
        return sm::vec3(0, 1, 0);
    }
}

Utility::PointStatus 
Utility::CalcPointPlaneStatus(const sm::Plane& plane, const sm::vec3& pos)
{
    const float dist = plane.GetDistance(pos);
    if (dist > POINT_STATUS_EPSILON) {
        return PointStatus::Above;
    } else if (dist < -POINT_STATUS_EPSILON) {
        return PointStatus::Below;
    } else {
        return PointStatus::Inside;
    }
}

Utility::FaceStatus 
Utility::CalcFacePlaneStatus(const Polyhedron::Face& face, const sm::Plane& plane)
{
    size_t above = 0, below = 0, inside = 0;

    auto count_loop = [&](const loop3& loop) 
    {
        auto first_e = loop.edge;
        auto curr_e = first_e;
        do {
            auto st = CalcPointPlaneStatus(plane, curr_e->vert->position);
            switch (st)
            {
            case PointStatus::Above:
                ++above;
                break;
            case PointStatus::Below:
                ++below;
                break;
            case PointStatus::Inside:
                ++inside;
                break;
            default:
                assert(0);
            }

            curr_e = curr_e->next;
        } while (first_e != curr_e);
    };

    if (face.border) {
        count_loop(*face.border);
    }
    for (auto& hole : face.holes) {
        count_loop(*hole);
    }

    if (above != 0 && below == 0) {
        return Utility::FaceStatus::Above;
    } else if (below != 0 && above == 0) {
        return Utility::FaceStatus::Below;
    } else if (inside != 0 && above == 0 && below == 0) {
        return Utility::FaceStatus::Inside;
    } else if (above != 0 && below != 0) {
        return Utility::FaceStatus::Cross;
    } else {
        assert(0);
        return Utility::FaceStatus::Cross;
    }
}

}