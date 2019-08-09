#pragma once

namespace he
{

// T has the member: linked_prev, linked_next
template<typename T>
class DoublyLinkedList
{
public:
    ~DoublyLinkedList();

    DoublyLinkedList& Append(T* item);

    T* Head() const { return m_head; }
    size_t Size() const { return m_size; }

private:
    void Clear();

    bool Check();
    bool CheckLinks();
    bool CheckSize();

private:
    T*     m_head = nullptr;
    size_t m_size = 0;

}; // DoublyLinkedList

}

#include "halfedge/DoublyLinkedList.inl"