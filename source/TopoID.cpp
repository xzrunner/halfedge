#include "halfedge/TopoID.h"

#include <boost/functional/hash.hpp>

namespace he
{

TopoID::TopoID(size_t id)
{
    m_path.push_back(id);

    UpdateUID();
}

void TopoID::Append(size_t id)
{
    m_path.push_back(id);

    UpdateUID();
}

void TopoID::Offset(size_t off)
{
    for (auto& p : m_path) {
        p += off;
    }

    UpdateUID();
}

void TopoID::Replace(size_t from, size_t to)
{
    bool dirty = false;
    for (auto& id : m_path)
    {
        if (id == from) {
            id = to;
            dirty = true;
        }
    }
    if (dirty) {
        UpdateUID();
    }
}

void TopoID::UpdateUID()
{
    m_uid = 0xffffffff;
    for (auto& id : m_path) {
        boost::hash_combine(m_uid, id);
    }
}

}