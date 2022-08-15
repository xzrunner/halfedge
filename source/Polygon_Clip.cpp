#include "halfedge/Polygon.h"

#include <SM_Rect.h>
#include <SM_Test.h>

namespace
{

sm::rect calc_aabb(const he::DoublyLinkedList<he::vert2>& verts)
{
	sm::rect aabb;

	auto head = verts.Head();
	if (head) {
		auto v = head;
		do {
			aabb.Combine(v->position);
			v = v->linked_next;
		} while (v != head);
	}

	return aabb;
}

//std::pair<bool, he::edge2*> FindInitialIntersectingEdge(const std::vector<sm::vec2>& polyline, const he::DoublyLinkedList<he::edge2>& edges)
//{
//
// 
//}

//bool IsIntersect(const sm::vec2& p0, const sm::vec2& p1, const sm::vec2& p2, const he::Polygon::Face& face)
//{
//	face.border
//}

bool clip(const he::Polygon::Face& face, const std::vector<sm::vec2>& polyline)
{
//	face.border->edge
	return false;
}

}

namespace he
{

bool Polygon::Clip(const std::vector<sm::vec2>& polyline)
{
	auto aabb = calc_aabb(m_verts);
	if (!sm::is_rect_intersect_polyline(aabb, polyline, false)) {
		return false;
	}

	bool ret = false;

	for (auto& face : m_faces) {
		if (clip(face, polyline)) {
			ret = true;
		}
	}

	return ret;
}

}