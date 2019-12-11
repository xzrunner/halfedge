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
    auto first_face = m_faces.Head();
    auto curr_face = first_face;
    do {
        sm::Plane plane;
        Utility::face_to_plane(*curr_face, plane);
        const float dist = plane.GetDistance(pos);
        if (dist > POINT_STATUS_EPSILON) {
            return false;
        }

        curr_face = curr_face->linked_next;
    } while (curr_face != first_face);

    return true;
}

}