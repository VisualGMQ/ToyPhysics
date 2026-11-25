#pragma once
#include "toy_physics/common/common.hpp"
#include "toy_physics/common/tight_pool.hpp"
#include "toy_physics/geometry/bounding_box.hpp"

#include <functional>
#include <span>

namespace toy_physics {

class Shape;

class BroadPhase {
public:
    enum class IndexSlot {
        SceneQuery,
        Simulation,
    };

    explicit BroadPhase(IndexSlot slot = IndexSlot::SceneQuery)
        : m_index_slot{slot} {}

    virtual ~BroadPhase() = default;

    static void AttachIndexTo(Shape& shape, TightPoolID index, IndexSlot slot);
    static TightPoolID FetchIndexFrom(const Shape& shape, IndexSlot slot);

    virtual void AddObjects(std::span<Shape> shapes) = 0;
    virtual void RemoveObjects(std::span<Shape> shapes) = 0;
    virtual void ApplyModify() = 0;
    [[nodiscard]] virtual bool ShouldRebuild() const = 0;

    /**
     * 遍历所有与 aabb 相交的 Shape，对每个调用 fn。
     *
     * 这里只做 BroadPhase 层的过滤，NarrowPhase 由调用方在 fn 里做。
     */
    virtual void DoFilter(const AABB& aabb,
                          const std::function<void(Shape*)>& fn) const = 0;

    // 便捷重载：一批 Shape*（Shape 在此处是前置声明，定义放在 bp.cpp）
    void AddObjects(std::span<Shape*> shapes);
    void RemoveObjects(std::span<Shape*> shapes);

    [[nodiscard]] IndexSlot GetIndexSlot() const { return m_index_slot; }

protected:
    IndexSlot m_index_slot;
};

}  // namespace toy_physics
