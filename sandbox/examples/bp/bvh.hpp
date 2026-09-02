#pragma once
#include "example.hpp"
#include "examples/bp/util.hpp"
#include "raylib.h"
#include "toy_physics/bp/bvh_topdown.hpp"
#include "toy_physics/geometry/geometry.hpp"

#include <memory>
#include <vector>

class BvhExample : public IExample {
public:
    using IExample::IExample;

    void OnUpdate(float delta_time) override;
    void OnRender3D(float delta_time) override;
    void OnRender2D(float delta_time) override;

private:
    struct RenderItem {
        toy_physics::Geometry::Type m_type{};
        ::Vector3 m_center{};
        ::Vector3 m_rotation{};
        ::Vector3 m_half_extent{};
        float m_radius{};
        float m_height{};
    };

    void rebuildScene();
    void rebuildBvh();
    void applySplitPolicy();
    [[nodiscard]] toy_physics::BVHSplitPolicy getSplitPolicy() const;

    bool m_initialized = false;
    bool m_draw_shapes = true;
    bool m_draw_bvh_aabbs = true;
    float m_rebuild_time_ms = 0.f;
    int m_split_policy_index = 1;
    int m_applied_policy_index = 1;

    std::unique_ptr<
        toy_physics::AABBBroadPhase<toy_physics::BVHBuildPolicy::TopDown>>
        m_broad_phase;
    std::vector<RenderItem> m_render_items;
    bp_util::BvhTreeView m_tree;
    bp_util::ResizablePanel m_panel;
    size_t m_node_count = 0;
    size_t m_leaf_count = 0;
};
