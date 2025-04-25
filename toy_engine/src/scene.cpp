#include "scene.hpp"

namespace toy_physics {

Scene::Scene() {
    std::shared_ptr<Body> body = std::make_shared<Body>();
    body->m_shape = std::make_shared<SphereShape>(1.0);
    m_bodies.emplace_back(std::move(body));
}

void Scene::Update(float delta_time) {
    for (auto& body : m_bodies) {
        body->m_position += body->m_linear_vel * delta_time;
    }
}

const std::vector<std::shared_ptr<Body>>& Scene::GetBodies() const {
    return m_bodies;
}

}  // namespace toy_physics