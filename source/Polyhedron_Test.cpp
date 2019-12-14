#include "halfedge/Polyhedron.h"
#include "halfedge/Utility.h"

namespace
{

const float POINT_STATUS_EPSILON = 0.0001f;

}

namespace he
{

bool Polyhedron::IsContain(const sm::vec3& pos) const
{
    auto first_l = m_loops.Head();
    auto curr_l = first_l;
    do {
        sm::Plane plane;
        Utility::LoopToPlane(*curr_l, plane);
        const float dist = plane.GetDistance(pos);
        if (dist > POINT_STATUS_EPSILON) {
            return false;
        }

        curr_l = curr_l->linked_next;
    } while (curr_l != first_l);

    return true;
}

}