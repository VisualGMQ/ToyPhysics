#include "nickel/common/macro.hpp"
#include "nickel/main_entry/runtime.hpp"
#include "nickel/nickel.hpp"
#include "scene.hpp"

using namespace toy_physics;

class Application : public nickel::Application {
public:
    void OnInit() override {
        auto& ctx = nickel::Context::GetInst();
        nickel::FlyCamera& camera = (nickel::FlyCamera&)ctx.GetCamera();

        camera.MoveTo(nickel::Vec3{0, 10, 10});
        camera.SetPitch(nickel::Degrees{-45});

        auto& mouse = ctx.GetDeviceManager().GetMouse();
        mouse.RelativeMode(true);
        mouse.Show(false);

        loadModels();
    }

    void OnUpdate() override {
        auto& ctx = nickel::Context::GetInst();
        auto& keyboard = ctx.GetDeviceManager().GetKeyboard();
        if (keyboard.GetKey(nickel::input::Key::Escape).IsPressed()) {
            ctx.Exit();
        }

        updateCamera();
        drawGrid();
        drawPhysicsObjects();
    }

private:
    Scene m_scene;
    nickel::graphics::GLTFModel m_unit_box;
    nickel::graphics::GLTFModel m_unit_sphere;

    void loadModels() {
        auto& ctx = nickel::Context::GetInst();
        auto engine_relative_path = ctx.GetEngineRelativePath();

        auto& mgr = ctx.GetGLTFManager();

        mgr.Load(engine_relative_path /
                 "engine/assets/models/unit_box/unit_box.gltf");
        mgr.Load(engine_relative_path /
                 "engine/assets/models/unit_sphere/unit_sphere.gltf");

        m_unit_box = mgr.Find(
            (engine_relative_path / "engine/assets/models/unit_box/unit_box")
                .ToString());
        m_unit_sphere = mgr.Find(
            (engine_relative_path / "engine/assets/models/unit_sphere/unit_sphere")
                .ToString());
    }

    void updateCamera() {
        auto& ctx = nickel::Context::GetInst();
        nickel::FlyCamera& camera =
            static_cast<nickel::FlyCamera&>(ctx.GetCamera());
        auto& device_mgr = ctx.GetDeviceManager();
        auto& keyboard = device_mgr.GetKeyboard();
        auto& mouse = device_mgr.GetMouse();

        NICKEL_RETURN_IF_FALSE(mouse.IsRelativeMode());

        constexpr float speed = 0.01f;

        if (auto btn = keyboard.GetKey(nickel::input::Key::W);
            btn.IsPressing()) {
            camera.MoveForward(speed);
        }
        if (auto btn = keyboard.GetKey(nickel::input::Key::S);
            btn.IsPressing()) {
            camera.MoveForward(-speed);
        }
        if (auto btn = keyboard.GetKey(nickel::input::Key::A);
            btn.IsPressing()) {
            camera.MoveRightLeft(-speed);
        }
        if (auto btn = keyboard.GetKey(nickel::input::Key::D);
            btn.IsPressing()) {
            camera.MoveRightLeft(speed);
        }
        if (auto btn = keyboard.GetKey(nickel::input::Key::R);
            btn.IsPressing()) {
            camera.MoveUpDown(speed);
        }
        if (auto btn = keyboard.GetKey(nickel::input::Key::F);
            btn.IsPressing()) {
            camera.MoveUpDown(-speed);
        }

        constexpr nickel::Degrees rot_radians{0.1};
        auto offset = mouse.GetOffset();

        camera.AddYaw(-offset.x * rot_radians);
        camera.AddPitch(-offset.y * rot_radians);
    }

    void drawGrid() {
        auto& ctx = nickel::Context::GetInst().GetGraphicsContext();
        ctx.SetClearColor(nickel::Color{0.1, 0.1, 0.1, 1.0f});

        constexpr uint8_t HalfLineNum = 10;
        nickel::Color color{0.7, 0.7, 0.7, 1.0};
        for (int i = -HalfLineNum; i <= HalfLineNum; i++) {
            if (i == 0) {
                continue;
            }
            nickel::graphics::Vertex vertices[] = {
                nickel::graphics::Vertex{nickel::Vec3(i, 0,  HalfLineNum),
                                         color},
                nickel::graphics::Vertex{nickel::Vec3(i, 0, -HalfLineNum),
                                         color}
            };
            ctx.DrawLineList(vertices);
        }

        for (int i = -HalfLineNum; i <= HalfLineNum; i++) {
            if (i == 0) {
                continue;
            }
            nickel::graphics::Vertex vertices[] = {
                nickel::graphics::Vertex{ nickel::Vec3(HalfLineNum, 0, i),
                                         color},
                nickel::graphics::Vertex{nickel::Vec3(-HalfLineNum, 0, i),
                                         color}
            };
            ctx.DrawLineList(vertices);
        }

        // draw axis
        {
            nickel::graphics::Vertex vertices[] = {
                nickel::graphics::Vertex{          nickel::Vec3(0, 0, 0),
                                         nickel::Color{1, 0, 0, 1}},
                nickel::graphics::Vertex{nickel::Vec3(HalfLineNum, 0, 0),
                                         nickel::Color{1, 0, 0, 1}}
            };
            ctx.DrawLineList(vertices);
        }
        {
            nickel::graphics::Vertex vertices[] = {
                nickel::graphics::Vertex{nickel::Vec3(0,           0, 0),
                                         nickel::Color{0, 1, 0, 1}},
                nickel::graphics::Vertex{nickel::Vec3(0, HalfLineNum, 0),
                                         nickel::Color{0, 1, 0, 1}}
            };
            ctx.DrawLineList(vertices);
        }
        {
            nickel::graphics::Vertex vertices[] = {
                nickel::graphics::Vertex{nickel::Vec3(0, 0,           0),
                                         nickel::Color{0, 0, 1, 1}},
                nickel::graphics::Vertex{nickel::Vec3(0, 0, HalfLineNum),
                                         nickel::Color{0, 0, 1, 1}}
            };
            ctx.DrawLineList(vertices);
        }
    }

    void drawPhysicsObjects() {
        auto& bodies = m_scene.GetBodies();
        for (auto& body : bodies) {
            auto type = body->m_shape->GetType();
            switch (type) {
                case Shape::Type::Sphere: {
                    SphereShape* sphere =
                        static_cast<SphereShape*>(body->m_shape.get());
                    nickel::Vec3 center_of_mass =
                        body->GetCenterOfMassWorldSpace();
                    nickel::Transform transform;
                    transform.scale = {sphere->m_radius, sphere->m_radius,
                                       sphere->m_radius};
                    transform.p = center_of_mass;
                    transform.q = body->m_rotation;
                    nickel::Context::GetInst().GetGraphicsContext().DrawModel(
                        transform, m_unit_sphere);
                } break;
            }
        }
    }
};

NICKEL_RUN_APP(Application);