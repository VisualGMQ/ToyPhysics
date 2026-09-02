#pragma once
#include "bp.hpp"
#include "toy_physics/bp/bvh.hpp"
#include "toy_physics/common/tight_pool.hpp"
#include "toy_physics/geometry/bounding_box.hpp"

#include <array>
#include <unordered_map>
#include <unordered_set>

namespace toy_physics {

class Shape;

/**
 * Node of the incremental BVH: keeps explicit child/parent ids into the
 * node pool and explicit object ids into the object pool.
 */
struct IncrementalNode {
    static constexpr uint32_t PrimMaxNum = BVH_OBJ_MAX_NUM;

    bool m_is_leaf{false};
    uint32_t m_object_count{0};
    TightPoolID m_parent{InvalidTightPoolID};
    BoundingBox m_aabb;
    std::array<TightPoolID, 2> m_child{InvalidTightPoolID, InvalidTightPoolID};
    std::array<TightPoolID, PrimMaxNum> m_objects{InvalidTightPoolID};
};

/**
 * PrunerPool for the incremental BVH: additionally tracks which node every
 * object lives in so removals can also drop the object from its node.
 */
template <>
class PrunerPool<BVHBuildPolicy::Incremental>
    : public TightPool<Shape*, BoundingBox> {
public:
    using NodePool = TightPool<IncrementalNode>;

    explicit PrunerPool(NodePool& node_pool) : m_node_pool{node_pool} {}

    /**
     * Remove an object and also erase it from the node that owns it.
     *
     * @return the id of the node that held the object, or
     * InvalidTightPoolID when the object was not tracked.
     */
    TightPoolID Remove(TightPoolID obj_id);

    void SetObjectNode(TightPoolID obj_id, TightPoolID node_id);

    [[nodiscard]] TightPoolID GetObjectNode(TightPoolID obj_id) const;

    /**
     * Pre-allocate the object-node tracking table before a rebuild burst.
     */
    void ReserveObjectNodeMap(size_t count) { m_obj_node_map.reserve(count); }

private:
    std::unordered_map<TightPoolID, TightPoolID> m_obj_node_map;
    NodePool& m_node_pool;
};

/**
 * using BVH, BV is AABB, binary space partition
 *
 * Incremental building: objects are queued by Add/Remove and only inserted
 * or removed from the tree in TryRebuild. Nodes keep explicit child/parent
 * ids into the node pool and explicit object ids into the object pool.
 */
template <>
class AABBBroadPhase<BVHBuildPolicy::Incremental> : public BroadPhase {
public:
    using Node = IncrementalNode;
    using NodePool = TightPool<Node>;
    using Pool = PrunerPool<BVHBuildPolicy::Incremental>;

    explicit AABBBroadPhase(BVHSplitPolicy split_policy = BVHSplitPolicy::SAH);

    void AddObjects(std::span<Shape> shapes) override;
    void RemoveObjects(std::span<Shape> shapes) override;

    void ApplyModify() override;
    [[nodiscard]] bool ShouldRebuild() const override;

    /**
     * Debug inspection interfaces, const views of internal data.
     */
    [[nodiscard]] const std::vector<Node>& GetNodes() const {
        return m_node_pool.GetPayloads<Node>();
    }

    [[nodiscard]] const std::vector<Shape*>& GetPrimitives() const {
        return m_pool.GetPayloads<Shape*>();
    }

    [[nodiscard]] size_t GetPrimitiveCount() const {
        return m_pool.GetPayloadSize();
    }

    [[nodiscard]] const Shape* GetPrimitiveByOrder(uint32_t order_index) const {
        TOY_ASSERT(order_index < m_pool.GetPayloadSize());
        return m_pool.GetPayloads<Shape*>()[order_index];
    }

    [[nodiscard]] TightPoolID GetRoot() const { return m_root; }

    [[nodiscard]] bool HasNode(TightPoolID node_id) const {
        return m_node_pool.Has(node_id);
    }

    [[nodiscard]] const Node& GetNode(TightPoolID node_id) const {
        return m_node_pool.Get<Node>(node_id);
    }

    [[nodiscard]] const Shape* GetPrimitiveByID(TightPoolID obj_id) const {
        return m_pool.Get<Shape*>(obj_id);
    }

    [[nodiscard]] TightPoolID GetObjectNode(TightPoolID obj_id) const {
        return m_pool.GetObjectNode(obj_id);
    }

private:
    NodePool m_node_pool;
    Pool m_pool;
    TightPoolID m_root{InvalidTightPoolID};

    std::unordered_set<Shape*> m_pending_add_objects;
    std::unordered_set<Shape*> m_pending_remove_objects;

    BVHSplitPolicy m_split_policy;

    void insertObject(TightPoolID obj_id);
    void splitLeaf(TightPoolID node_id, TightPoolID obj_id);
    void mergeEmptyLeaf(TightPoolID node_id);
    void refit(TightPoolID node_id);
};

}  // namespace toy_physics
