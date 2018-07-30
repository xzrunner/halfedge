#pragma once

#include <SM_Vector.h>
#include <SM_Plane.h>

#include <memory>
#include <vector>

namespace he
{

struct Vertex;
struct Edge;
struct Face;

using VertexPtr = std::shared_ptr<Vertex>;
using EdgePtr   = std::shared_ptr<Edge>;
using FacePtr   = std::shared_ptr<Face>;

struct Vertex
{
	Vertex(sm::vec3 position)
		: position(position)
	{}

	sm::vec3 position;

	std::vector<EdgePtr> output;
	std::vector<EdgePtr> input;

}; // Vertex

struct Edge
{
	Edge(VertexPtr origin, FacePtr face)
		: origin(origin), face(face) {}

	VertexPtr origin = nullptr;
	FacePtr   face = nullptr;

	EdgePtr twin = nullptr;

	EdgePtr prev = nullptr;
	EdgePtr next = nullptr;

}; // Edge

const EdgePtr& ConnectEdge(const EdgePtr& prev, const EdgePtr& next);

struct Face
{
	EdgePtr start_edge = nullptr;

	void GetBorder(std::vector<sm::vec3>& border) const;

	void GetPlane(sm::Plane& plane) const;

}; // Face

}