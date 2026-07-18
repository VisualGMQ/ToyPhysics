#include "examples/render_test/render_test.hpp"
#include "context.hpp"

void RenderTestExample::OnUpdate(float delta_time) {}

void RenderTestExample::OnRender3D(float delta_time) {
    m_ctx.DrawBox(Vector3{0, 0, 0}, Vector3{0, 0, 0}, Vector3{2, 1, 1}, WHITE);
    m_ctx.DrawSphere(Vector3{5, 0, 0}, Vector3{0, 0, 0}, 1.0f, WHITE);
    m_ctx.DrawCylinder(Vector3{-5, 0, 0}, Vector3{0, 0, 0}, 2.0f, 1.0f, WHITE);
    m_ctx.DrawCapsule(Vector3{0, 0, -5}, Vector3{90, 0, 0}, 2.0f, 1.0f, WHITE);
}