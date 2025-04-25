#pragma once
#include "body.hpp"

namespace toy_physics {

class Scene {
public:
    Scene();
    void Update(float delta_time);
    const std::vector<std::shared_ptr<Body>>& GetBodies() const;

private:
    std::vector<std::shared_ptr<Body>> m_bodies;
};

}