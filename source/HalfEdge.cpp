#include "halfedge/HalfEdge.h"

namespace he
{

const EdgePtr& ConnectEdge(const EdgePtr& prev, const EdgePtr& next)
{
	prev->next = next;
	next->prev = prev;
	next->origin->output.push_back(next);
	next->origin->input.push_back(prev);
	return next;
}

void Face::GetBorder(std::vector<sm::vec3>& border) const
{
	auto ptr = start_edge;
	while (true)
	{
		border.push_back(ptr->origin->position);

		ptr = ptr->next;
		if (ptr == start_edge) {
			break;
		}
	}
}

}