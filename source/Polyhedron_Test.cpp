#include "halfedge/Polyhedron.h"
#include "halfedge/Utility.h"

#include <SM_Calc.h>

namespace
{

bool is_pos_in_loop(const sm::vec3& pos, const he::loop3& loop)
{
    sm::Plane plane;
    he::Utility::LoopToPlane(loop, plane);
    const float dist = plane.GetDistance(pos);
    return dist < he::Utility::POINT_STATUS_EPSILON;
}

}

namespace he
{

bool Polyhedron::IsContain(const sm::vec3& pos) const
{
    for (auto& face : m_faces)
    {
        if (!is_pos_in_loop(pos, *face.border)) {
            continue;
        }

        bool contain = true;
        for (auto& hole : face.holes) {
            if (!is_pos_in_loop(pos, *face.border)) {
                contain = false;
                break;
            }
        }
        if (contain) {
            return true;
        }
    }

    return false;
}

}