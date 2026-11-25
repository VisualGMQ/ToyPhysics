#pragma once

#include "toy_physics/common/transform.hpp"
#include "toy_physics/geometry/geometry.hpp"
#include "toy_physics/lowlevel/gjk.hpp"

namespace toy_physics {

Vector3 getGeomSupport(const Geometry& geom, const Transform& trans,
                       Vector3 dir);

class GeomSupportFunction : public SupportFunction {
public:
    GeomSupportFunction(const Geometry& geom, const Transform& trans)
        : m_geom{geom}, m_trans{trans} {}

    [[nodiscard]] Vector3 operator()(Vector3 dir) const override {
        return getGeomSupport(m_geom, m_trans, dir);
    }

private:
    const Geometry& m_geom;
    const Transform& m_trans;
};

}  // namespace toy_physics
