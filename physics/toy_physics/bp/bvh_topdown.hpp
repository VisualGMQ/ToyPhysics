#pragma once

#include "toy_physics/bp/bp.hpp"
#include "toy_physics/bp/bvh.hpp"
#include "toy_physics/common/check.hpp"
#include "toy_physics/common/common.hpp"
#include "toy_physics/common/tight_pool.hpp"
#include "toy_physics/geometry/bounding_box.hpp"

namespace toy_physics {

class Shape;

/**
 * PrunerPool for the top-down BVH: TightPool with object partition support.
 */
template <>
class PrunerPool<BVHBuildPolicy::TopDown>
    : public TightPool<Shape*, BoundingBox> {
public:
    /**
     * Median partition over the ids in [offset, offset + count): elements
     * [offset, offset + mid) get the smallest centroids.
     *
     * @return mid, the number of ids in the left part
     */
    uint32_t MedianPartition(std::vector<uint32_t>& ids, uint32_t offset,
                             uint32_t count, uint8_t axis);

    /**
     * Binned SAH partition over the ids in [offset, offset + count).
     *
     * @return mid, the number of ids in the left part
     */
    uint32_t SAHPartition(std::vector<uint32_t>& ids, uint32_t offset,
                          uint32_t count, uint32_t bucket_num, uint8_t axis);

private:
    struct Bucket {
        uint32_t m_count = 0;
        BoundingBox m_aabb{};
        bool m_has_aabb = false;
    };

    struct Side {
        uint32_t m_count = 0;
        BoundingBox m_aabb{};
        bool m_has_aabb = false;

        void Add(const Bucket& bucket) {
            m_count += bucket.m_count;
            m_aabb = m_has_aabb ? m_aabb.Merge(bucket.m_aabb) : bucket.m_aabb;
            m_has_aabb = true;
        }
    };

    /**
     * Reorder the ids in [offset, offset + count) by a permutation of local
     * indices. Applied in-place by following the permutation cycles.
     */
    void permuteIDs(std::vector<uint32_t>& ids, uint32_t offset, uint32_t count,
                    const std::vector<uint32_t>& perm);

    [[nodiscard]] auto& AABBs();
    [[nodiscard]] const auto& AABBs() const;

    std::vector<uint32_t> m_perm;
    std::vector<uint32_t> m_perm_inv;
    std::vector<uint8_t> m_perm_visited;
    std::vector<real> m_partition_keys;
    std::vector<real> m_sah_keys;
    std::vector<Bucket> m_sah_buckets;
    std::vector<uint32_t> m_sah_prim_bucket;
    std::vector<Side> m_sah_right;
};

/**
 * using BVH, BV is AABB, binary space partition
 */
template <>
class AABBBroadPhase<BVHBuildPolicy::TopDown> : public BroadPhase {
public:
    struct Node {
        struct Data {
            static constexpr uint32_t LeafMask = 1;

            // When is leaf, bits means:
            static constexpr uint32_t PrimMaxNum = 0xF;
            static constexpr uint32_t PrimNumMask = PrimMaxNum << 1;
            static constexpr uint32_t PrimOffset = ~(LeafMask | PrimNumMask);

            // When not leaf, bits means:
            static constexpr uint32_t PositiveChildMask = ~LeafMask;

            [[nodiscard]] bool IsLeaf() const { return m_data & LeafMask; }

            [[nodiscard]] uint8_t GetPrimNum() const {
                return (m_data & PrimNumMask) >> 1;
            }

            [[nodiscard]] uint32_t GetPrimOffset() const {
                return (m_data & PrimOffset) >> 5;
            }

            [[nodiscard]] uint32_t GetPositiveChild() const {
                return (m_data & PositiveChildMask) >> 1;
            }

            [[nodiscard]] uint32_t GetNegativeChild() const {
                return GetPositiveChild() + 1;
            }

            void SetPositiveChild(uint32_t index) {
                TOY_ASSERT((index & ~(PositiveChildMask >> 1)) == 0);
                m_data = (index << 1) & PositiveChildMask;
            }

            void SetAsLeaf(uint32_t prim_offset, uint32_t prim_num) {
                TOY_ASSERT((prim_offset & ~(PrimOffset >> 5)) == 0);
                TOY_ASSERT((prim_num & ~PrimMaxNum) == 0);
                m_data = LeafMask | ((prim_num & PrimMaxNum) << 1) |
                         ((prim_offset << 5) & PrimOffset);
            }

        private:
            uint32_t m_data{0};
        };

        Data m_data;
        BoundingBox m_aabb;
    };

    explicit AABBBroadPhase(BVHSplitPolicy split_policy = BVHSplitPolicy::SAH);

    void AddObjects(std::span<Shape> shapes) override;
    void RemoveObjects(std::span<Shape> shapes) override;

    void ApplyModify() override;
    [[nodiscard]] bool ShouldRebuild() const override;

    /**
     * Debug inspection interfaces, const views of internal data.
     */
    [[nodiscard]] const std::vector<Node>& GetNodes() const { return m_nodes; }

    [[nodiscard]] const std::vector<Shape*>& GetPrimitives() const {
        return m_pool.GetPayloads<Shape*>();
    }

    [[nodiscard]] size_t GetPrimitiveCount() const {
        return m_pool.GetPayloadSize();
    }

    [[nodiscard]] const Shape* GetPrimitiveByOrder(uint32_t order_index) const {
        TOY_ASSERT(order_index < m_ids.size());
        return m_pool.GetPayloads<Shape*>()[m_ids[order_index]];
    }

private:
    using Pool = PrunerPool<BVHBuildPolicy::TopDown>;

    Pool m_pool;
    std::vector<Node> m_nodes;
    std::vector<uint32_t> m_ids;

    std::vector<Shape*> m_pending_add_objects;
    std::vector<Shape*> m_pending_remove_objects;

    BVHSplitPolicy m_split_policy;

    void topdownRebuild();
    void topdownRebuildRecursive(uint32_t node_index,
                                 uint32_t& node_using_count,
                                 uint32_t prim_offset, uint32_t prim_count);
    void refreshPool();
};

}  // namespace toy_physics