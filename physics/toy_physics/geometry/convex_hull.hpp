#pragma once

#include "toy_physics/common/storage.hpp"
#include "toy_physics/geometry/geometry.hpp"

namespace toy_physics {

class ConvexData {
public:
    std::vector<Vector3> m_vertices;
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
