#include "halfedge/Utility.h"

#include <SM_Calc.h>

#include <set>

namespace he
{

void Utility::face_to_vertices(const face3& face, std::vector<sm::vec3>& border)

void Utility::FaceToVertices(const face3& face, std::vector<sm::vec3>& border)
{
    auto first_edge = face.edge;
    auto curr_edge = first_edge;
    do {
        border.push_back(curr_edge->vert->position);
        curr_edge = curr_edge->next;
    } while (curr_edge != first_edge);
}

void Utility::FaceToPlane(const face3& face, sm::Plane& plane)
{
    auto curr_edge = face.edge;
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