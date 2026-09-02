#include "toy_physics/bp/bvh_topdown.hpp"
#include "toy_physics/common/check.hpp"
#include "toy_physics/shape.hpp"
#include <algorithm>
#include <limits>
#include <numeric>
#include <span>
#include <utility>
#include <vector>

namespace toy_physics {

auto& PrunerPool<BVHBuildPolicy::TopDown>::AABBs() {
    return std::get<1>(m_payloads);
}

const auto& PrunerPool<BVHBuildPolicy::TopDown>::AABBs() const {
    return std::get<1>(m_payloads);
}

uint32_t PrunerPool<BVHBuildPolicy::TopDown>::MedianPartition(
    std::vector<uint32_t>& ids, uint32_t offset, uint32_t count, uint8_t axis) {
    TOY_ASSERT(offset + count <= ids.size());

    uint32_t mid = count / 2;

    // precompute every centroid once so the nth_element comparator only
    // compares cached floats instead of re-evaluating it per comparison
    const auto& aabbs = AABBs();
    m_partition_keys.resize(count);
    for (uint32_t i = 0; i < count; ++i) {
        const BoundingBox& bv = aabbs[ids[offset + i]];
        m_partition_keys[i] =
            (bv.m_max[axis] + bv.m_min[axis]) * static_cast<real>(0.5);
    }

    m_perm.resize(count);
    std::iota(m_perm.begin(), m_perm.begin() + count, offset);
    std::nth_element(m_perm.begin(), m_perm.begin() + mid,
                     m_perm.begin() + count, [&](uint32_t a, uint32_t b) {
                         return m_partition_keys[a - offset] <
                                m_partition_keys[b - offset];
                     });

    permuteIDs(ids, offset, count, m_perm);
    return mid;
}

uint32_t PrunerPool<BVHBuildPolicy::TopDown>::SAHPartition(
    std::vector<uint32_t>& ids, uint32_t offset, uint32_t count,
    uint32_t bucket_num, uint8_t axis) {
    TOY_ASSERT(offset + count <= ids.size());
    TOY_ASSERT(bucket_num >= 2);
    TOY_ASSERT(count >= 2);

    const auto& aabbs = AABBs();
    m_sah_keys.resize(count);
    real key_min = std::numeric_limits<real>::max();
    real key_max = std::numeric_limits<real>::lowest();
    for (uint32_t i = 0; i < count; ++i) {
        const BoundingBox& bv = aabbs[ids[offset + i]];
        m_sah_keys[i] =
            (bv.m_max[axis] + bv.m_min[axis]) * static_cast<real>(0.5);
        key_min = std::min(key_min, m_sah_keys[i]);
        key_max = std::max(key_max, m_sah_keys[i]);
    }

    // degenerate key range: no useful seam, fall back to median split
    if (key_max - key_min <= std::numeric_limits<real>::epsilon()) {
        return MedianPartition(ids, offset, count, axis);
    }

    m_sah_buckets.resize(bucket_num);
    for (Bucket& bucket : m_sah_buckets) {
        bucket.m_count = 0;
        bucket.m_has_aabb = false;
    }
    m_sah_prim_bucket.resize(count);

    real bucket_size = (key_max - key_min) / static_cast<real>(bucket_num);
    for (uint32_t i = 0; i < count; ++i) {
        uint32_t b =
            static_cast<uint32_t>((m_sah_keys[i] - key_min) / bucket_size);
        if (b >= bucket_num) {
            b = bucket_num - 1;
        }
        m_sah_prim_bucket[i] = b;

        Bucket& bucket = m_sah_buckets[b];
        const BoundingBox& aabb = aabbs[ids[offset + i]];
        if (bucket.m_has_aabb) {
            bucket.m_aabb = bucket.m_aabb.Merge(aabb);
        } else {
            bucket.m_aabb = aabb;
            bucket.m_has_aabb = true;
        }
        ++bucket.m_count;
    }

    // accumulate merged AABBs and counts from the right (suffix):
    // m_sah_right[b] merges buckets [b, bucket_num)
    m_sah_right.resize(bucket_num);
    for (Side& side : m_sah_right) {
        side = Side{};
    }
    for (uint32_t b = bucket_num; b-- > 0;) {
        if (b + 1 < bucket_num) {
            m_sah_right[b] = m_sah_right[b + 1];
        }
        if (m_sah_buckets[b].m_count > 0) {
            m_sah_right[b].Add(m_sah_buckets[b]);
        }
    }

    // walk the seams forward, maintaining the left side incrementally
    // and remembering only the cheapest seam
    Side left;
    uint32_t best = 0;
    real best_cost = std::numeric_limits<real>::max();
    bool found = false;
    for (uint32_t i = 0; i + 1 < bucket_num; ++i) {
        if (m_sah_buckets[i].m_count > 0) {
            left.Add(m_sah_buckets[i]);
        }
        if (left.m_count == 0 || m_sah_right[i + 1].m_count == 0) {
            continue;
        }
        real cost = left.m_aabb.Area() * static_cast<real>(left.m_count) +
                    m_sah_right[i + 1].m_aabb.Area() *
                        static_cast<real>(m_sah_right[i + 1].m_count);
        if (cost < best_cost) {
            best_cost = cost;
            best = i;
            found = true;
        }
    }

    // no seam splits both sides non-empty: fall back to median split
    if (!found) {
        return MedianPartition(ids, offset, count, axis);
    }

    // left side = ids in buckets [0, best]
    m_perm.resize(count);
    std::iota(m_perm.begin(), m_perm.begin() + count, offset);
    auto mid_it = std::stable_partition(
        m_perm.begin(), m_perm.begin() + count,
        [&](uint32_t idx) { return m_sah_prim_bucket[idx - offset] <= best; });
    uint32_t mid = static_cast<uint32_t>(mid_it - m_perm.begin());

    permuteIDs(ids, offset, count, m_perm);
    return mid;
}

void PrunerPool<BVHBuildPolicy::TopDown>::permuteIDs(
    std::vector<uint32_t>& ids, uint32_t offset, uint32_t count,
    const std::vector<uint32_t>& perm) {
    // inverse permutation: the element at local position i moves to
    // m_perm_inv[i]
    m_perm_inv.resize(count);
    for (uint32_t i = 0; i < count; ++i) {
        m_perm_inv[perm[i] - offset] = i;
    }

    m_perm_visited.assign(count, 0);
    for (uint32_t start = 0; start < count; ++start) {
        if (m_perm_visited[start]) {
            continue;
        }
        uint32_t j = start;
        auto saved = std::move(ids[offset + j]);
        do {
            m_perm_visited[j] = 1;
            uint32_t k = m_perm_inv[j];
            std::swap(saved, ids[offset + k]);
            j = k;
        } while (j != start);
    }
}

AABBBroadPhase<BVHBuildPolicy::TopDown>::AABBBroadPhase(
    BVHSplitPolicy split_policy)
    : m_split_policy{split_policy} {}

void AABBBroadPhase<BVHBuildPolicy::TopDown>::AddObjects(
    std::span<Shape> shapes) {
    for (auto& shape : shapes) {
        m_pending_add_objects.push_back(&shape);
    }
}

void AABBBroadPhase<BVHBuildPolicy::TopDown>::RemoveObjects(
    std::span<Shape> shapes) {
    for (auto& shape : shapes) {
        m_pending_remove_objects.push_back(&shape);
    }
}

void AABBBroadPhase<BVHBuildPolicy::TopDown>::ApplyModify() {
    if (!ShouldRebuild()) {
        return;
    }

    topdownRebuild();
}

bool AABBBroadPhase<BVHBuildPolicy::TopDown>::ShouldRebuild() const {
    return !m_pending_add_objects.empty() || !m_pending_remove_objects.empty();
}

void AABBBroadPhase<BVHBuildPolicy::TopDown>::topdownRebuild() {
    m_nodes.clear();
    refreshPool();

    if (m_pool.IsEmpty()) {
        return;
    }

    m_nodes.resize(1);
    uint32_t node_using_count = 1;
    topdownRebuildRecursive(0, node_using_count, 0, m_pool.GetPayloadSize());
}

void AABBBroadPhase<BVHBuildPolicy::TopDown>::topdownRebuildRecursive(
    uint32_t node_index, uint32_t& node_using_count, uint32_t prim_offset,
    uint32_t prim_count) {
    const auto& aabbs = m_pool.GetPayloads<BoundingBox>();

    Node& node = m_nodes[node_index];
    node.m_aabb = aabbs[m_ids[prim_offset]];
    for (uint32_t i = prim_offset + 1; i < prim_offset + prim_count; ++i) {
        node.m_aabb = node.m_aabb.Merge(aabbs[m_ids[i]]);
    }

    if (prim_count <= Node::Data::PrimMaxNum) {
        node.m_data.SetAsLeaf(prim_offset, prim_count);
        return;
    }

    auto range =
        m_nodes[node_index].m_aabb.m_max - m_nodes[node_index].m_aabb.m_min;
    uint8_t axis;
    range.maxCoeff(&axis);

    uint32_t mid;
    switch (m_split_policy) {
        case BVHSplitPolicy::Median:
            mid = m_pool.MedianPartition(m_ids, prim_offset, prim_count, axis);
            break;
        case BVHSplitPolicy::SAH:
            mid = m_pool.SAHPartition(m_ids, prim_offset, prim_count, 16, axis);
            break;
    }

    if (m_nodes.size() < node_using_count + 2) {
        m_nodes.resize(node_using_count + 2);
    }
    uint32_t positive_index = node_using_count;
    node_using_count += 2;

    m_nodes[node_index].m_data.SetPositiveChild(positive_index);

    topdownRebuildRecursive(positive_index, node_using_count, prim_offset + mid,
                            prim_count - mid);
    topdownRebuildRecursive(positive_index + 1, node_using_count, prim_offset,
                            mid);
}

void AABBBroadPhase<BVHBuildPolicy::TopDown>::refreshPool() {
    for (auto shape : m_pending_add_objects) {
        if (!shape) {
            continue;
        }
        AttachIndexTo(*shape, m_pool.Add(shape, shape->GetBoundingBox()));
    }
    m_pending_add_objects.clear();

    for (auto shape : m_pending_remove_objects) {
        if (!shape) {
            continue;
        }
        m_pool.Remove(FetchIndexFrom(*shape));
    }
    m_pending_remove_objects.clear();

    // BVH primitive order starts as pool order; partitions only permute it
    m_ids.resize(m_pool.GetPayloadSize());
    std::iota(m_ids.begin(), m_ids.end(), 0u);
}

}  // namespace toy_physics
