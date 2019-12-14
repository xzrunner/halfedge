#include "halfedge/Polyhedron.h"
#include "halfedge/Utility.h"

namespace
{

bool IsPolyhedronClosed(const he::Polyhedron& poly)
{
    auto first_edge = poly.GetEdges().Head();
    auto curr_edge = first_edge;
    do {
        if (curr_edge->twin == nullptr) {
            return false;
        }

        curr_edge = curr_edge->linked_next;
    } while (curr_edge != first_edge);

    return true;
}

std::shared_ptr<he::Polyhedron>
DoIntersect(const he::Polyhedron& poly0, const he::Polyhedron& poly1)
{
    assert(IsPolyhedronClosed(poly0));

    auto ret = std::make_shared<he::Polyhedron>(poly0);

    auto first_face = poly1.GetFaces().Head();
    auto curr_face = first_face;
    do {
        sm::Plane plane;
        he::Utility::FaceToPlane(*curr_face, plane);

        bool succ = ret->Clip(plane, he::Polyhedron::KeepType::KeepBelow, true);
        if (ret->GetFaces().Size() == 0) {
            return ret;
        }

        curr_face = curr_face->linked_next;
    } while (curr_face != first_face);

    return ret;
}

void DoSubtract(std::vector<he::PolyhedronPtr>& result, const std::vector<he::PolyhedronPtr>& fragments,
                const he::face3* curr_face, const he::face3* first_face)
{
    if (fragments.empty()) {
        return;
    }

    sm::Plane plane;
    he::Utility::FaceToPlane(*curr_face, plane);

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

    bool is_closed0 = IsPolyhedronClosed(*this);
    bool is_closed1 = IsPolyhedronClosed(other);
    if (!is_closed0 && !is_closed1) {
        return nullptr;
    }

    if (is_closed0) {
        return DoIntersect(*this, other);
    } else {
        return DoIntersect(other, *this);
    }
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