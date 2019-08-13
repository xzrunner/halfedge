#pragma once

#include <assert.h>

namespace he
{

template <typename T>
DoublyLinkedList<T>::~DoublyLinkedList()
{
    Clear();

    assert(Check());
}

template <typename T>
DoublyLinkedList<T>&
DoublyLinkedList<T>::Append(T* item)
{
    if (m_head == nullptr)
    {
        assert(m_size == 0);
        m_head = item;
        m_head->linked_prev = m_head->linked_next = m_head;
        m_size = 1;
    }
    else
    {
        item->linked_prev = m_head->linked_prev;
        item->linked_next = m_head;
        m_head->linked_prev->linked_next = item;
        m_head->linked_prev = item;
        ++m_size;
    }

    assert(Check());

    return *this;
}

template <typename T>
T* DoublyLinkedList<T>::Remove(T* item)
{
    assert(m_size > 0);

    if (m_head == item) {
        m_head = item->linked_next;
    }

    auto next_item = item->linked_next;

    item->linked_next->linked_prev = item->linked_prev;
    item->linked_prev->linked_next = item->linked_next;
    --m_size;

    assert(Check());

    return next_item;
}

template <typename T>
void DoublyLinkedList<T>::Clear()
{
    if (!m_head) {
        assert(m_size == 0);
        return;
    }

    auto item = m_head;
    do {
        auto next = item->linked_next;
        delete item;
        item = next;
    } while (item != m_head);

    m_head = nullptr;
    m_size = 0;

    assert(Check());
}

template <typename T>
bool DoublyLinkedList<T>::Check()
{
    return CheckLinks() && CheckSize();
}

template <typename T>
bool DoublyLinkedList<T>::CheckLinks()
{
    if (!m_head) {
        return true;
    }

    auto item = m_head;
    do {
        auto next = item->linked_next;
        if (!next) {
            return false;
        }
        if (next->linked_prev != item) {
            return false;
        }
        item = next;
    } while (item != m_head);

    return true;
}

template <typename T>
bool DoublyLinkedList<T>::CheckSize()
{
    if (!m_head) {
        return m_size == 0;
    }

    size_t size = 0;
    auto item = m_head;
    do {
        item = item->linked_next;
        ++size;
    } while (item != m_head);

    return m_size == size;
}

}