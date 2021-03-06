#pragma once

#include <vector>

namespace he
{

class TopoID
{
public:
    TopoID() {}
    TopoID(size_t id);

    bool operator == (const TopoID& id) const {
        return m_path == id.m_path;
    }

    void Append(size_t id);
    void Pop();

    size_t UID() const { return m_uid; }

    bool Empty() const { return m_path.empty(); }

    void Offset(size_t off);
    void Replace(size_t from, size_t to);

    auto& Path() const { return m_path; }

    void MakeInvalid() { m_path = { 0xffffffff }; }
    bool IsValid() const {
        return !(m_path.size() == 1 && m_path[0] == 0xffffffff);
    }

private:
    void UpdateUID();

private:
    std::vector<size_t> m_path;

    size_t m_uid = 0xffffffff;

}; // TopoID

}