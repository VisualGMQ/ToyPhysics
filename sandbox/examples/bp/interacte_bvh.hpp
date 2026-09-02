#pragma once
#include "example.hpp"
#include "examples/bp/util.hpp"
#include "raylib.h"
#include "toy_physics/bp/bvh_topdown.hpp"
#include "toy_physics/geometry/geometry.hpp"

#include <memory>
#include <vector>

class InteracteBvhExample : public IExample {
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
        ::Vector3 m_half_extent{1, 1, 1};
        float m_radius{1};
        float m_height{2};
    };

    std::unique_ptr<toy_physics::Shape> createShapeFromItem(
        const RenderItem& item) const;
    void applyTransform(size_t index);
    void addShape(RenderItem item);
    void deleteSelectedShape();
    void recreateSelectedShape();
    void rebuildBvh();
    void applySplitPolicy();
    [[nodiscard]] toy_physics::BVHSplitPolicy getSplitPolicy() const;
    void pickShape(::Vector2 mouse);
    float raycastShape(const ::Ray& ray, const RenderItem& item) const;
    void renderCreateSection(float x, float& y, float panelW);
    void renderSelectedSection(float x, float& y, float panelW);

    bool m_initialized = false;
    int m_selected_shape = -1;
    int m_split_policy_index = 1;
    int m_applied_policy_index = 1;
    size_t m_node_count = 0;
    size_t m_leaf_count = 0;

    std::unique_ptr<
        toy_physics::AABBBroadPhase<toy_physics::BVHBuildPolicy::TopDown>>
        m_broad_phase;
    std::vector<std::unique_ptr<toy_physics::Shape>> m_shapes;
    std::vector<std::unique_ptr<toy_physics::Shape>> m_graveyard;
    std::vector<RenderItem> m_render_items;
    bp_util::BvhTreeView m_tree;
    bp_util::ResizablePanel m_panel;
};
