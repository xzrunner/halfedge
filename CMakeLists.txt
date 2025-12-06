cmake_minimum_required(VERSION 3.16)

project(halfedge)

add_definitions(
    -DNO_BOOST
)

################################################################################
# Source groups
################################################################################
set(2d
    "include/halfedge/Polygon.h"
    "source/Polygon.cpp"
    "source/Polygon_Clip.cpp"
    "source/Polygon_Edit.cpp"
)
source_group("2d" FILES ${2d})

set(3d
    "include/halfedge/Polyhedron.h"
    "include/halfedge/Polyline.h"
    "source/Polyhedron.cpp"
    "source/Polyhedron_Boolean.cpp"
    "source/Polyhedron_Build.cpp"
    "source/Polyhedron_Clip.cpp"
    "source/Polyhedron_Edit.cpp"
    "source/Polyhedron_Test.cpp"
    "source/Polyline.cpp"
)
source_group("3d" FILES ${3d})

set(dataset
    "include/halfedge/DoublyLinkedList.h"
    "include/halfedge/DoublyLinkedList.inl"
    "include/halfedge/HalfEdge.h"
    "include/halfedge/HalfEdge.inl"
    "include/halfedge/TopoID.h"
    "source/HalfEdge.cpp"
    "source/TopoID.cpp"
)
source_group("dataset" FILES ${dataset})

set(utility
    "include/halfedge/noncopyable.h"
    "include/halfedge/typedef.h"
    "include/halfedge/Utility.h"
    "include/halfedge/Utility.inl"
    "source/Utility.cpp"
)
source_group("utility" FILES ${utility})

set(ALL_FILES
    ${2d}
    ${3d}
    ${dataset}
    ${utility}
)

################################################################################
# Target
################################################################################
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/external/sm)

add_library(${PROJECT_NAME} STATIC ${ALL_FILES})
