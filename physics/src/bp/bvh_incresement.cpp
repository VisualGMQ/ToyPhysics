#include "toy_physics/bp/bvh_incresement.hpp"
#include "toy_physics/shape.hpp"

#include <algorithm>
#include <limits>
#include <span>

namespace toy_physics {

AABBBroadPhase<BVHBuildPolicy::Incremental>::AABBBroadPhase(
    BVHSplitPolicy split_policy)
    : m_pool{m_node_pool}, m_split_policy{split_policy} {}

void AABBBroadPhase<BVHBuildPolicy::Incremental>::AddObjects(
    std::span<Shape> shapes) {
    for (auto& shape : shapes) {
        Shape* ptr = &shape;
        m_pending_remove_objects.erase(ptr);
        m_pending_add_objects.insert(ptr);
    }
}

void AABBBroadPhase<BVHBuildPolicy::Incremental>::RemoveObjects(
    std::span<Shape> shapes) {
    for (auto& shape : shapes) {
        Shape* ptr = &shape;
        m_pending_add_objects.erase(ptr);
        m_pending_remove_objects.insert(ptr);
    }
}

bool AABBBroadPhase<BVHBuildPolicy::Incremental>::ShouldRebuild() const {
    return !m_pending_add_objects.empty() || !m_pending_remove_objects.empty();
}

void AABBBroadPhase<BVHBuildPolicy::Incremental>::ApplyModify() {
    if (!ShouldRebuild()) {
        return;
    }

    m_pool.ReserveObjectNodeMap(m_pool.GetPayloadSize() +
                                m_pending_add_objects.size());

    // apply removals first: drop the object from its node, then either refit
    // the node or collapse it away
    for (Shape* shape : m_pending_remove_objects) {
        if (!shape) {
            continue;
        }
        TightPoolID obj_id = FetchIndexFrom(*shape);
        if (!m_pool.Has(obj_id) || m_pool.Get<Shape*>(obj_id) != shape) {
            continue;
        }
        TightPoolID node_id = m_pool.Remove(obj_id);
        if (node_id == InvalidTightPoolID) {
            continue;
        }
        TOY_ASSERT(m_node_pool.Has(node_id));
        Node& node = m_node_pool.Get<Node>(node_id);
        if (node.m_object_count > 0) {
            refit(node_id);
        } else {
            mergeEmptyLeaf(node_id);
        }
    }
    m_pending_remove_objects.clear();

    // then apply additions: insert into the existing tree
    for (Shape* shape : m_pending_add_objects) {
        if (!shape) {
            continue;
        }
        TightPoolID existing_id = FetchIndexFrom(*shape);
        if (m_pool.Has(existing_id) &&
            m_pool.Get<Shape*>(existing_id) == shape) {
            // the shape is already tracked (e.g. it moved): refresh the
            // cached BV in place and refit its subtree
            m_pool.Get<BoundingBox>(existing_id) = shape->GetBoundingBox();
            TightPoolID node_id = m_pool.GetObjectNode(existing_id);
            if (node_id != InvalidTightPoolID) {
                refit(node_id);
            }
            continue;
        }
        TightPoolID obj_id = m_pool.Add(shape, shape->GetBoundingBox());
        if (obj_id == InvalidTightPoolID) {
            continue;
        }
        AttachIndexTo(*shape, obj_id);
        insertObject(obj_id);
    }
    m_pending_add_objects.clear();
}

TightPoolID PrunerPool<BVHBuildPolicy::Incremental>::Remove(
    TightPoolID obj_id) {
    TightPoolID node_id = InvalidTightPoolID;
    auto it = m_obj_node_map.find(obj_id);
    if (it != m_obj_node_map.end()) {
        node_id = it->second;
        m_obj_node_map.erase(it);
        if (node_id != InvalidTightPoolID && m_node_pool.Has(node_id)) {
            IncrementalNode& node = m_node_pool.Get<IncrementalNode>(node_id);
            TOY_ASSERT(node.m_is_leaf);
            for (uint32_t i = 0; i < node.m_object_count; ++i) {
                if (node.m_objects[i] == obj_id) {
                    node.m_objects[i] = node.m_objects[node.m_object_count - 1];
                    node.m_objects[node.m_object_count - 1] =
                        InvalidTightPoolID;
                    --node.m_object_count;
                    break;
                }
            }
        }
    }
    TightPool<Shape*, BoundingBox>::Remove(obj_id);
    return node_id;
}

void PrunerPool<BVHBuildPolicy::Incremental>::SetObjectNode(
    TightPoolID obj_id, TightPoolID node_id) {
    m_obj_node_map[obj_id] = node_id;
}

TightPoolID PrunerPool<BVHBuildPolicy::Incremental>::GetObjectNode(
    TightPoolID obj_id) const {
    auto it = m_obj_node_map.find(obj_id);
    return it == m_obj_node_map.end() ? InvalidTightPoolID : it->second;
}

void AABBBroadPhase<BVHBuildPolicy::Incremental>::insertObject(
    TightPoolID obj_id) {
    const BoundingBox& obj_bv = m_pool.Get<BoundingBox>(obj_id);

    if (m_root == InvalidTightPoolID || !m_node_pool.Has(m_root)) {
        TightPoolID id = m_node_pool.Add(Node{});
        Node& node = m_node_pool.Get<Node>(id);
        node.m_is_leaf = true;
        node.m_objects[0] = obj_id;
        node.m_object_count = 1;
        node.m_aabb = obj_bv;
        m_pool.SetObjectNode(obj_id, id);
        m_root = id;
        return;
    }

    TightPoolID cur = m_root;
    while (true) {
        Node& node = m_node_pool.Get<Node>(cur);
        if (node.m_is_leaf) {
            if (node.m_object_count < Node::PrimMaxNum) {
                node.m_objects[node.m_object_count] = obj_id;
                ++node.m_object_count;
                node.m_aabb = node.m_aabb.Merge(obj_bv);
                m_pool.SetObjectNode(obj_id, cur);
                refit(node.m_parent);
                return;
            }
            splitLeaf(cur, obj_id);
            return;
        }

        // descend to the child whose BV center is closer to the object's
        Vector3 obj_center = (obj_bv.m_min + obj_bv.m_max) * real(0.5);
        const Node& c0 = m_node_pool.Get<Node>(node.m_child[0]);
        const Node& c1 = m_node_pool.Get<Node>(node.m_child[1]);
        Vector3 c0_center = (c0.m_aabb.m_min + c0.m_aabb.m_max) * real(0.5);
        Vector3 c1_center = (c1.m_aabb.m_min + c1.m_aabb.m_max) * real(0.5);
        cur = (obj_center - c0_center).squaredNorm() <=
                      (obj_center - c1_center).squaredNorm()
                  ? node.m_child[0]
                  : node.m_child[1];
    }
}

void AABBBroadPhase<BVHBuildPolicy::Incremental>::splitLeaf(
    TightPoolID node_id, TightPoolID obj_id) {
    Node& node = m_node_pool.Get<Node>(node_id);
    TOY_ASSERT(node.m_is_leaf);
    TOY_ASSERT(node.m_object_count == Node::PrimMaxNum);

    constexpr uint32_t count = Node::PrimMaxNum + 1;
    std::array<TightPoolID, count> obj_ids;
    for (uint32_t i = 0; i < Node::PrimMaxNum; ++i) {
        obj_ids[i] = node.m_objects[i];
    }
    obj_ids[Node::PrimMaxNum] = obj_id;

    TightPoolID parent_id = node.m_parent;
    BoundingBox total = node.m_aabb.Merge(m_pool.Get<BoundingBox>(obj_id));
    Vector3 range = total.m_max - total.m_min;
    uint8_t axis;
    range.maxCoeff(&axis);

    // small top-down split: sort centroids along the longest axis, then cut
    // at the median (or at the cheapest SAH seam)
    std::sort(obj_ids.begin(), obj_ids.end(),
              [&](TightPoolID a, TightPoolID b) {
                  const BoundingBox& aabb_a = m_pool.Get<BoundingBox>(a);
                  const BoundingBox& aabb_b = m_pool.Get<BoundingBox>(b);
                  return (aabb_a.m_min[axis] + aabb_a.m_max[axis]) <
                         (aabb_b.m_min[axis] + aabb_b.m_max[axis]);
              });

    uint32_t mid = count / 2;
    if (m_split_policy == BVHSplitPolicy::SAH) {
        std::array<BoundingBox, count> suffix;
        suffix[count - 1] = m_pool.Get<BoundingBox>(obj_ids[count - 1]);
        for (uint32_t i = count - 1; i-- > 0;) {
            suffix[i] =
                suffix[i + 1].Merge(m_pool.Get<BoundingBox>(obj_ids[i]));
        }
        BoundingBox left_bv;
        bool has_left = false;
        real best_cost = std::numeric_limits<real>::max();
        for (uint32_t i = 1; i < count; ++i) {
            const BoundingBox& bv = m_pool.Get<BoundingBox>(obj_ids[i - 1]);
            left_bv = has_left ? left_bv.Merge(bv) : bv;
            has_left = true;
            real cost = left_bv.Area() * static_cast<real>(i) +
                        suffix[i].Area() * static_cast<real>(count - i);
            if (cost < best_cost) {
                best_cost = cost;
                mid = i;
            }
        }
    }
    TOY_ASSERT(mid > 0 && mid < count);

    // allocating children may invalidate the node reference taken above
    TightPoolID left_id = m_node_pool.Add(Node{});
    TightPoolID right_id = m_node_pool.Add(Node{});
    Node& left = m_node_pool.Get<Node>(left_id);
    Node& right = m_node_pool.Get<Node>(right_id);
    left.m_is_leaf = true;
    right.m_is_leaf = true;
    left.m_parent = node_id;
    right.m_parent = node_id;

    for (uint32_t i = 0; i < mid; ++i) {
        TightPoolID id = obj_ids[i];
        left.m_objects[i] = id;
        m_pool.SetObjectNode(id, left_id);
        const BoundingBox& bv = m_pool.Get<BoundingBox>(id);
        left.m_aabb = (i == 0) ? bv : left.m_aabb.Merge(bv);
    }
    left.m_object_count = mid;
    for (uint32_t i = mid; i < count; ++i) {
        uint32_t r = i - mid;
        TightPoolID id = obj_ids[i];
        right.m_objects[r] = id;
        m_pool.SetObjectNode(id, right_id);
        const BoundingBox& bv = m_pool.Get<BoundingBox>(id);
        right.m_aabb = (r == 0) ? bv : right.m_aabb.Merge(bv);
    }
    right.m_object_count = count - mid;

    Node& refreshed = m_node_pool.Get<Node>(node_id);
    refreshed.m_is_leaf = false;
    refreshed.m_object_count = 0;
    refreshed.m_objects.fill(InvalidTightPoolID);
    refreshed.m_child = {left_id, right_id};
    refreshed.m_aabb = left.m_aabb.Merge(right.m_aabb);

    refit(parent_id);
}

void AABBBroadPhase<BVHBuildPolicy::Incremental>::mergeEmptyLeaf(
    TightPoolID node_id) {
    Node& node = m_node_pool.Get<Node>(node_id);
    TOY_ASSERT(node.m_is_leaf && node.m_object_count == 0);

    TightPoolID parent_id = node.m_parent;
    if (parent_id == InvalidTightPoolID) {
        m_node_pool.Remove(node_id);
        m_root = InvalidTightPoolID;
        return;
    }

    Node& parent = m_node_pool.Get<Node>(parent_id);
    TOY_ASSERT(!parent.m_is_leaf);
    TightPoolID sibling_id =
        (parent.m_child[0] == node_id) ? parent.m_child[1] : parent.m_child[0];
    TOY_ASSERT(sibling_id != InvalidTightPoolID);
    TightPoolID grand_id = parent.m_parent;

    // the sibling takes the parent's place, then both freed nodes go away
    if (grand_id == InvalidTightPoolID) {
        m_root = sibling_id;
    } else {
        Node& grand = m_node_pool.Get<Node>(grand_id);
        if (grand.m_child[0] == parent_id) {
            grand.m_child[0] = sibling_id;
        } else {
            TOY_ASSERT(grand.m_child[1] == parent_id);
            grand.m_child[1] = sibling_id;
        }
    }
    m_node_pool.Get<Node>(sibling_id).m_parent = grand_id;

    m_node_pool.Remove(node_id);
    m_node_pool.Remove(parent_id);

    // the sibling's own BV is unchanged by the pull-up, refit from the
    // grandparent upward instead
    refit(grand_id);
}

void AABBBroadPhase<BVHBuildPolicy::Incremental>::refit(TightPoolID node_id) {
    while (node_id != InvalidTightPoolID) {
        Node& node = m_node_pool.Get<Node>(node_id);
        BoundingBox bv;
        if (node.m_is_leaf) {
            TOY_ASSERT(node.m_object_count > 0);
            bv = m_pool.Get<BoundingBox>(node.m_objects[0]);
            for (uint32_t i = 1; i < node.m_object_count; ++i) {
                bv = bv.Merge(m_pool.Get<BoundingBox>(node.m_objects[i]));
            }
        } else {
            bv = m_node_pool.Get<Node>(node.m_child[0])
                     .m_aabb.Merge(
                         m_node_pool.Get<Node>(node.m_child[1]).m_aabb);
        }
        // an unchanged BV means no ancestor can change either
        if (bv.m_min == node.m_aabb.m_min && bv.m_max == node.m_aabb.m_max) {
            break;
        }
        node.m_aabb = bv;
        node_id = node.m_parent;
    }
}

}  // namespace toy_physics
