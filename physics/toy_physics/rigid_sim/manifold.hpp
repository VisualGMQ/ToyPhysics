#pragma once

#include "toy_physics/common/transform.hpp"
#include "toy_physics/common/math.hpp"
#include "toy_physics/geometry/geometry.hpp"
#include <array>

namespace toy_physics {

class Shape;

struct Manifold {
    static constexpr int kMaxContacts = 4;

    std::array<Vector3, kMaxContacts> m_contacts1{};
    std::array<Vector3, kMaxContacts> m_contacts2{};
    Vector4 m_depth = Vector4::Zero();  // contact point pair depth
    uint8_t m_contact_num{0};

    Vector3 m_separate_normal = Vector3::Zero();  // from body1 to body2
    real m_penetration_depth{0};

    const Shape* m_shape1{};
    const Shape* m_shape2{};

    [[nodiscard]] bool IsValid() const { return m_contact_num > 0; }
};

/*
 * @return false when the geometries are separated or the query failed.
 */
bool ComputeManifold(const Shape& shape1, const Shape& shape2,
                     Manifold* out_manifold);

/**
 * @param out_penetrate witness1 - witness2
 */
bool ComputePenetration(const Geometry& geom1, const Transform& trans1,
                        const Geometry& geom2, const Transform& trans2,
                        Vector3* out_penetrate, Vector3* out_witness1,
                        Vector3* out_witness2);

}  // namespace toy_physics
