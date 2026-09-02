#pragma once
#include "toy_physics/common/check.hpp"
#include "toy_physics/common/common.hpp"
#include "toy_physics/common/log.hpp"
#include "toy_physics/common/noncopyable.hpp"

#include <cstddef>
#include <memory>
#include <new>
#include <vector>

namespace toy_physics {

template <typename T>
class Storage : public NonCopyable {
public:
    template <typename>
    friend struct StorageElem;

    template <typename... Args>
    T* Create(Args&&... args) {
        Chunk* chunk;
        uint32_t index;
        if (m_unuse_head != InvalidUnuseHead) {
            index = m_unuse_head;
            chunk = m_chunks[index].get();
            m_unuse_head = chunk->m_id.GetNext();
        } else {
            if (m_chunks.size() >= Chunk::ID::IndexMask) {
                LOGW("create elem failed: storage pool is full "
                     "(uint32_t limit reached)");
                return nullptr;
            }
            index = static_cast<uint32_t>(m_chunks.size());
            auto new_chunk = std::make_unique<Chunk>(0);
            chunk = new_chunk.get();
            m_chunks.push_back(std::move(new_chunk));
        }
        try {
            new (chunk->m_payload) T(std::forward<Args>(args)...);
        } catch (const std::exception& e) {
            LOGE("create elem failed: constructor error: {}", e.what());
            pushUnused(index);
            return nullptr;
        } catch (...) {
            LOGE("create elem failed: constructor error");
            pushUnused(index);
            return nullptr;
        }
        chunk->m_id.SetAsUse(index);
        m_using_count++;
        return reinterpret_cast<T*>(chunk->m_payload);
    }

    void Destroy(T* payload) {
        if (!payload) {
            return;
        }
        Chunk* chunk = reinterpret_cast<Chunk*>(payload);
        if (!chunk->m_id.IsUsing()) {
            return;
        }
        uint32_t index = chunk->m_id.GetIndex();
        if (index >= m_chunks.size()) {
            return;
        }
        reinterpret_cast<T*>(chunk->m_payload)->~T();
        m_using_count--;
        pushUnused(index);
    }

    T* Get(StorageID id) {
        uint32_t index = id & Chunk::ID::IndexMask;
        if (index >= m_chunks.size()) {
            return nullptr;
        }
        Chunk* chunk = m_chunks[index].get();
        if (!chunk->m_id.IsUsing()) {
            return nullptr;
        }
        return reinterpret_cast<T*>(chunk->m_payload);
    }

    void Clear() {
        for (uint32_t i = 0; i < m_chunks.size(); ++i) {
            Chunk* chunk = m_chunks[i].get();
            if (chunk->m_id.IsUsing()) {
                reinterpret_cast<T*>(chunk->m_payload)->~T();
            }
            chunk->m_id.SetAsUnuse(i + 1 == m_chunks.size() ? InvalidUnuseHead
                                                            : i + 1);
        }
        m_using_count = 0;
        m_unuse_head = m_chunks.empty() ? InvalidUnuseHead : 0;
    }

    ~Storage() { Clear(); }

private:
    struct Chunk {
        struct ID {
            static constexpr uint32_t UsingMask = 1 << 31;
            static constexpr uint32_t IndexMask = 0x7FFFFFFF;

            explicit ID(uint32_t data) : m_data{data} {}

            [[nodiscard]] bool IsUsing() const { return m_data & UsingMask; }

            [[nodiscard]] uint32_t GetIndex() const {
                return m_data & IndexMask;
            }

            void SetAsUse(uint32_t index) {
                TOY_ASSERT((index & UsingMask) == 0);
                m_data = UsingMask | index;
            }

            void SetAsUnuse(uint32_t next) {
                TOY_ASSERT((next & UsingMask) == 0);
                m_data = next;
            }

            [[nodiscard]] uint32_t GetNext() const {
                return m_data & IndexMask;
            }

            uint32_t m_data;
        };

        explicit Chunk(uint32_t id) : m_id{id} {}

        using Buffer = std::byte[sizeof(T)];

        alignas(T) Buffer m_payload;
        ID m_id;
    };

    constexpr static uint32_t InvalidUnuseHead = Chunk::ID::IndexMask;

    void pushUnused(uint32_t index) {
        m_chunks[index]->m_id.SetAsUnuse(m_unuse_head);
        m_unuse_head = index;
    }

    std::vector<std::unique_ptr<Chunk>> m_chunks;
    uint32_t m_using_count = 0;
    uint32_t m_unuse_head{InvalidUnuseHead};
};

/**
 * Only used for element managed by Storage<T>
 *
 * @see Storage<T>
 */
template <typename T>
struct StorageElem {
    [[nodiscard]] StorageID GetID() const {
        using chunk_type = Storage<T>::Chunk;
        auto chunk = reinterpret_cast<const chunk_type*>(this);
        return chunk->m_id.GetIndex();
    }
};

}  // namespace toy_physics
