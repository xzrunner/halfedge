#include "halfedge/Polyhedron.h"

namespace
{


void DoSubtract(std::vector<he::PolyhedronPtr>& result, const std::vector<he::PolyhedronPtr>& fragments,
                const he::Face* curr_face, const he::Face* first_face)
{
    if (fragments.empty()) {
        return;
    }

    sm::Plane plane;
    face_to_plane(*curr_face, plane);

    std::vector<he::PolyhedronPtr> back_frags;

    for (auto& frag : fragments)
    {
        auto frag_in_front = std::make_shared<he::Polyhedron>(*frag);
        if (frag_in_front->Clip(plane, he::Polyhedron::KeepType::KeepAbove, true)) {
            result.push_back(frag_in_front);
        }

        auto fragment_behind = std::make_shared<he::Polyhedron>(*frag);
        if (fragment_behind->Clip(plane, he::Polyhedron::KeepType::KeepBelow, true)) {
            back_frags.push_back(fragment_behind);
        }
    }

    curr_face = curr_face->linked_next;
    if (curr_face != first_face) {
        DoSubtract(result, back_frags, curr_face, first_face);
    }
}

}

namespace he
{

PolyhedronPtr Polyhedron::Union(const Polyhedron& other) const
{
    if (m_faces.Size() <= 3 || other.m_faces.Size() <= 3) {
        return nullptr;
    }

    return nullptr;
}

PolyhedronPtr Polyhedron::Intersect(const Polyhedron& other) const
{
    if (m_faces.Size() <= 3 || other.m_faces.Size() <= 3) {
        return nullptr;
    }

    auto ret = std::make_shared<Polyhedron>(other);

    auto first_face = m_faces.Head();
    auto curr_face = first_face;
    do {
        sm::Plane plane;
        face_to_plane(*curr_face, plane);
        ret->Clip(plane, KeepType::KeepBelow, true);
        if (ret->m_faces.Size() == 0) {
            return ret;
        }

        curr_face = curr_face->linked_next;
    } while (curr_face != first_face);

    return ret;
}

std::vector<PolyhedronPtr> Polyhedron::Subtract(const Polyhedron& subtrahend) const
{
    std::vector<PolyhedronPtr> ret;
    auto first_face = subtrahend.GetFaces().Head();
    DoSubtract(ret, { std::make_shared<Polyhedron>(*this) }, first_face, first_face);

//    return { Fuse(ret) };
    return ret;
}

}