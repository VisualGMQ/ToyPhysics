#pragma once

#include <functional>
#include <utility>

namespace toy_physics {

template <typename T, typename U>
struct UnorderedPair {
    T m_value1{};
    U m_value2{};

    [[nodiscard]] bool operator==(const UnorderedPair& o) const {
        return (o.m_value1 == m_value1 && o.m_value2 == m_value2) ||
               (o.m_value2 == m_value1 && o.m_value1 == m_value2);
    }
};

}  // namespace toy_physics

namespace std {

template <typename T, typename U>
struct hash<toy_physics::UnorderedPair<T, U>> {
    size_t operator()(const toy_physics::UnorderedPair<T, U>& pair) const {
        size_t h1 = hash<T>{}(pair.m_value1);
        size_t h2 = hash<U>{}(pair.m_value2);

        if (h2 < h1) {
            std::swap(h1, h2);
        }

        return h1 ^ (h2 + 0x9e3779b97f4a7c15ULL + (h1 << 6) + (h1 >> 2));
    }
};

}  // namespace std
