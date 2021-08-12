#include "halfedge/Polyhedron.h"
#include "halfedge/Utility.h"

#include <iterator>

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
    auto faces = poly1.GetFaces();
    if (faces.empty()) {
        return ret;
    }

    auto first_l = faces.front().border;
    auto curr_l = first_l;
    do {
        sm::Plane plane;
        he::Utility::LoopToPlane(*curr_l, plane);

        bool succ = ret->Clip(plane, he::Polyhedron::KeepType::KeepBelow, true);
        if (ret->GetFaces().empty()) {
            return ret;
        }

        curr_l = curr_l->linked_next;
    } while (curr_l != first_l);

    return ret;
}

void DoSubtract(std::vector<he::PolyhedronPtr>& result, const std::vector<he::PolyhedronPtr>& fragments,
                const he::loop3* curr_l, const he::loop3* first_l)
{
    if (fragments.empty()) {
        return;
    }

    sm::Plane plane;
    he::Utility::LoopToPlane(*curr_l, plane);

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

    curr_l = curr_l->linked_next;
    if (curr_l != first_l) {
        DoSubtract(result, back_frags, curr_l, first_l);
    }
}

}

namespace he
{

std::vector<PolyhedronPtr> Polyhedron::Union(const Polyhedron& other) const
{
    auto intersected = Intersect(other);
    if (!intersected || intersected->GetFaces().empty()) {
        return { std::make_shared<he::Polyhedron>(*this), std::make_shared<he::Polyhedron>(other) };
    }
 
    auto ret = Subtract(*intersected);

    auto polys = other.Subtract(*intersected);
    std::copy(polys.begin(), polys.end(), std::back_inserter(ret));

    return ret;
}

PolyhedronPtr Polyhedron::Intersect(const Polyhedron& other) const
{
    if (m_faces.size() <= 3 || other.m_faces.size() <= 3) {
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

    auto& faces = subtrahend.GetFaces();
    if (!faces.empty()) {
        auto first_l = faces.front().border;
        DoSubtract(ret, { std::make_shared<Polyhedron>(*this) }, first_l, first_l);
    }

//    return { Fuse(ret) };
    return ret;
}

}