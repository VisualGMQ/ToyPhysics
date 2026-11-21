#pragma once

#include "toy_physics/common/storage.hpp"
#include "toy_physics/geometry/geometry.hpp"
#include <span>
#include <vector>

namespace toy_physics {

class ConvexData {
public:
    explicit ConvexData(std::span<const Vector3> vertices);

    [[nodiscard]] std::span<const Vector3> GetVertices() const;
    [[nodiscard]] real GetVolume() const;
    [[nodiscard]] const Vector3& GetCenterOfMass() const;
    [[nodiscard]] const Vector3& GetUnitInertia() const;
    [[nodiscard]] const Matrix33& GetPrincipalAxes() const;

private:
    std::vector<Vector3> m_vertices;

    real m_volume{0};
    Vector3 m_center_of_mass{Vector3::Zero()};
    Vector3 m_unit_inertia{Vector3::Zero()};
    Matrix33 m_principal_axes{Matrix33::Identity()};

    void computeMassProperty();
};

class ConvexHullGeometry : public Geometry,
                           public StorageElem<ConvexHullGeometry> {
public:
    ConvexHullGeometry();
    explicit ConvexHullGeometry(const ConvexData&);

    [[nodiscard]] bool IsValid() const;

    [[nodiscard]] const ConvexData* GetConvex() const;

private:
    const ConvexData* m_convex{};
};

class ConvexFactory : public Storage<ConvexData> {};

}  // namespace toy_physics
