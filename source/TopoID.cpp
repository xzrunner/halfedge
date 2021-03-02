#include "halfedge/TopoID.h"

#ifndef NO_BOOST
#include <boost/functional/hash.hpp>
#endif // NO_BOOST

namespace
{

#ifdef NO_BOOST
template <class T>
inline void hash_combine(std::size_t& seed, const T& v)
{
    std::hash<T> hasher;
    seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}
#endif // NO_BOOST

}

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

void TopoID::Pop()
{
    if (!m_path.empty()) {
        m_path.pop_back();
        UpdateUID();
    }
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
#ifdef NO_BOOST
        hash_combine(m_uid, id);
#else
        boost::hash_combine(m_uid, id);
#endif // NO_BOOST
    }
}

}