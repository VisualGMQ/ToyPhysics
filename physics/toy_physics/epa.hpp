#pragma once
#include "toy_physics/math.hpp"

namespace toy_physics {

struct SupportFunction;
class GJK;

/**
 * Check origin in tetrahedron
 *
 * @return the obsolete point index, start with 1. 0 means check success
 */
int IsOriginInTetrahedron(const Vector3& p1, const Vector3& p2,
                          const Vector3& p3, const Vector3& p4);

bool CalcPenetrationDepth(GJK& gjk,
                          const toy_physics::SupportFunction& support1,
                          const SupportFunction& support2,
                          Vector3& out_penetration, Vector3& out_p1,
                          Vector3& out_p2, real tolerance = 1e-3);

[[nodiscard]] bool CalcPenetrationDepth(const SupportFunction& support1,
                                        const SupportFunction& support2,
                                        Vector3& out_penetration,
                                        Vector3& out_p1, Vector3& out_p2,
                                        real tolerance = 1e-3);

}  // namespace toy_physics
