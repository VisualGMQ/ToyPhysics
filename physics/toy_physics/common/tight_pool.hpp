#pragma once
#include "toy_physics/common/check.hpp"
#include "toy_physics/common/common.hpp"
#include "toy_physics/common/log.hpp"

#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

namespace toy_physics {

static constexpr TightPoolID InvalidTightPoolID = static_cast<TightPoolID>(
    std::numeric_limits<std::underlying_type_t<TightPoolID>>::max());

namespace detail {

template <typename T, typename... Rest>
struct IsTypeUnique : std::conjunction<std::negation<std::is_same<T, Rest>>...,
                                       IsTypeUnique<Rest...>> {};

template <typename T>
struct IsTypeUnique<T> : std::true_type {};

}  // namespace detail

/**
 * Tight pool storing payloads as multiple SoA (structure of arrays) columns.
 * Recommend to store object reference or move-freely objects.
 */
template <typename... Ts>
class TightPool {
public:
    static_assert(detail::IsTypeUnique<Ts...>::value,
                  "TightPool payload column types must be unique");

    /**
     * Add an element to every column.
     *
     * @note Will not check pool has element. If you push a duplicate element,
     * it will store it and return a different index
     */
    TightPoolID Add(const Ts&... args) noexcept {
        TOY_ASSERT(m_payload_size == m_index_id_map.size());

        uint32_t index;
        if (m_unuse_head != InvalidUnusedHead) {
            index = m_unuse_head;
            m_unuse_head = m_id_index_map[index].GetNext();
        } else {
            if (m_id_index_map.size() >= Guide::IndexMask) {
                LOGW("create elem failed: indexed pool is full "
                     "(uint32_t limit reached)");
                return InvalidTightPoolID;
            }
            index = static_cast<uint32_t>(m_id_index_map.size());
            m_id_index_map.emplace_back(0);
        }

        uint32_t pushed_count = 0;
        try {
            ((std::get<std::vector<Ts>>(m_payloads).emplace_back(args),
              ++pushed_count),
             ...);
        } catch (const std::exception& e) {
            LOGW("create elem failed: constructor error: {}", e.what());
            rollbackPushed(pushed_count);
            m_id_index_map[index].SetAsUnuse(m_unuse_head);
            m_unuse_head = index;
            return InvalidTightPoolID;
        } catch (...) {
            LOGW("create elem failed: constructor error");
            rollbackPushed(pushed_count);
            m_id_index_map[index].SetAsUnuse(m_unuse_head);
            m_unuse_head = index;
            return InvalidTightPoolID;
        }
        uint32_t p = static_cast<uint32_t>(m_payload_size++);
        m_index_id_map.push_back(static_cast<TightPoolID>(index));
        m_id_index_map[index].SetAsUse(p);
        return static_cast<TightPoolID>(index);
    }

    /**
     * Remove an element by id (swap-remove on every column).
     */
    void Remove(TightPoolID id) noexcept {
        TOY_ASSERT(m_payload_size == m_index_id_map.size());
        auto numeric_id = static_cast<std::underlying_type_t<TightPoolID>>(id);

        if (numeric_id >= m_id_index_map.size()) {
            return;
        }
        const Guide& id_data = m_id_index_map[numeric_id];
        if (!id_data.IsUsing()) {
            return;
        }
        uint32_t p = id_data.GetIndex();
        if (p >= m_payload_size) {
            return;
        }
        TOY_ASSERT(m_index_id_map[p] == id);

        uint32_t last = static_cast<uint32_t>(m_payload_size - 1);
        uint32_t last_id = static_cast<uint32_t>(m_index_id_map[last]);
        std::apply(
            [&](auto&... arrays) {
                ((std::swap(arrays[p], arrays[last])), ...);
            },
            m_payloads);
        std::apply([&](auto&... arrays) { ((arrays.pop_back()), ...); },
                   m_payloads);
        --m_payload_size;
        std::swap(m_index_id_map[p], m_index_id_map[last]);
        m_index_id_map.pop_back();
        m_id_index_map[last_id].SetAsUse(p);
        m_id_index_map[numeric_id].SetAsUnuse(m_unuse_head);
        m_unuse_head = numeric_id;
    }

    [[nodiscard]] bool Has(TightPoolID id) const {
        auto numeric_id = static_cast<std::underlying_type_t<TightPoolID>>(id);
        if (numeric_id >= m_id_index_map.size()) {
            return false;
        }
        const Guide& id_data = m_id_index_map[numeric_id];
        if (!id_data.IsUsing()) {
            return false;
        }
        uint32_t p = id_data.GetIndex();
        return p < m_payload_size && m_index_id_map[p] == id;
    }

    [[nodiscard]] bool IsEmpty() const { return m_payload_size == 0; }

    /**
     * Get the payload column of type T by id.
     */
    template <typename T>
    [[nodiscard]] T& Get(TightPoolID id) {
        return const_cast<T&>(std::as_const(*this).template Get<T>(id));
    }

    template <typename T>
    [[nodiscard]] const T& Get(TightPoolID id) const {
        return std::get<std::vector<T>>(m_payloads)[resolvePayloadIndex(id)];
    }

    /**
     * Get the whole payload tuple (all columns).
     */
    template <typename T>
    [[nodiscard]] const auto& GetPayloads() const {
        return std::get<std::vector<T>>(m_payloads);
    }

    [[nodiscard]] const std::vector<TightPoolID>& GetAllIDs() const {
        return m_index_id_map;
    }

    [[nodiscard]] size_t GetPayloadSize() const { return m_payload_size; }

protected:
    struct Guide {
        static constexpr uint32_t UsingMask = 1 << 31;
        static constexpr uint32_t IndexMask = 0x7FFFFFFF;

        explicit Guide(uint32_t data) : m_data{data} {}

        [[nodiscard]] bool IsUsing() const { return m_data & UsingMask; }

        [[nodiscard]] uint32_t GetIndex() const { return m_data & IndexMask; }

        void SetAsUse(uint32_t index) {
            TOY_ASSERT((index & UsingMask) == 0);
            m_data = UsingMask | index;
        }

        void SetAsUnuse(uint32_t next) {
            TOY_ASSERT((next & UsingMask) == 0);
            m_data = next;
        }

        [[nodiscard]] uint32_t GetNext() const { return m_data & IndexMask; }

        uint32_t m_data;
    };

    static constexpr uint32_t InvalidUnusedHead = Guide::IndexMask;

    std::tuple<std::vector<Ts>...> m_payloads;
    std::vector<Guide> m_id_index_map;
    std::vector<TightPoolID> m_index_id_map;
    uint32_t m_unuse_head{InvalidUnusedHead};
    size_t m_payload_size{0};

    /**
     * Pop the columns that were already pushed when an emplace_back threw.
     */
    void rollbackPushed(uint32_t pushed_count) {
        std::apply(
            [&](auto&... arrays) {
                uint32_t i = 0;
                ((i++ < pushed_count ? (void)arrays.pop_back() : (void)0), ...);
            },
            m_payloads);
    }

    /**
     * Translate an id to its payload index.
     */
    [[nodiscard]] uint32_t resolvePayloadIndex(TightPoolID id) const {
        auto numeric_id = static_cast<std::underlying_type_t<TightPoolID>>(id);
        TOY_ASSERT(numeric_id < m_id_index_map.size());
        const Guide& guide = m_id_index_map[numeric_id];
        TOY_ASSERT(guide.IsUsing());
        return guide.GetIndex();
    }
};

}  // namespace toy_physics
