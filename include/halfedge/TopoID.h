#pragma once

#include <vector>

namespace he
{

class TopoID
{
public:
    TopoID() {}
    TopoID(size_t id);

    void Append(size_t id);

    size_t ID() const { return m_path.empty() ? 0xffffffff : m_path.back(); }
    size_t UID() const { return m_uid; }

    bool Empty() const { return m_path.empty(); }

    void Offset(size_t off);

    auto& Path() const { return m_path; }

private:
    void UpdateUID();

private:
    std::vector<size_t> m_path;

    size_t m_uid = 0xffffffff;

}; // TopoID

}