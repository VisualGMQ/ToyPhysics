#pragma once
#include <array>

template <typename T, size_t N>
class FixedStack {
public:
    [[nodiscard]] size_t GetSize() const { return m_top; }

    [[nodiscard]] static size_t GetCapacity() { return N; }

    void Push(const T& elem) {
        if (IsFull()) {
            return;
        }
        m_elems[m_top++] = elem;
    }

    void Push(T&& elem) {
        if (IsFull()) {
            return;
        }
        m_elems[m_top++] = std::move(elem);
    }

    void Pop() {
        if (IsEmpty()) {
            return;
        }

        m_top --;
    }

    const T& Top() const { return m_elems[m_top]; }
    T& Top() { return m_elems[m_top]; }

    [[nodiscard]] bool IsEmpty() const { return m_top == 0; }

    [[nodiscard]] bool IsFull() const { return m_top == N; }

private:
    std::array<T, N> m_elems;
    size_t m_top{};
};

template <typename T, size_t N>
class FixedRingQueue {
public:
    template <typename... Args>
    void Push(Args&&... args) {
        if (GetSize() == GetCapacity()) {
            return;
        }

        m_begin = next(m_begin);
        m_elems[m_begin] = T{std::forward<Args>(args)...};
    }

    const T& Top() const { return m_elems[m_begin]; }

    T& Top() { return m_elems[m_begin]; }

    void Pop() {
        if (IsEmpty()) {
            return;
        }

        m_begin = front(m_begin);
    }

    [[nodiscard]] bool IsEmpty() const { return m_begin == m_end; }

    [[nodiscard]] static constexpr size_t GetCapacity() { return N; }

    [[nodiscard]] constexpr size_t GetSize() const {
        if (m_begin >= m_end) {
            return GetSize() - (m_begin - m_end);
        }
        return m_end - m_begin;
    }

private:
    std::array<T, N> m_elems;
    size_t m_begin{};
    size_t m_end{};

    [[nodiscard]] size_t next(size_t idx) const {
        return (idx + 1) % m_elems.size();
    }

    [[nodiscard]] size_t front(size_t idx) const {
        if (idx == 0) {
            return N;
        }
        return idx - 1;
    }
};
