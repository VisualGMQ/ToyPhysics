#include "context.hpp"
#include "examples/render_test/render_test.hpp"
#include "examples/nearest_points/sphere.hpp"
#include "examples/nearest_points/capsule.hpp"
#include "examples/nearest_points/segment.hpp"
#include "examples/nearest_points/obb.hpp"
#include "examples/nearest_points/tetrahedron.hpp"
#include "examples/nearest_points/triangle.hpp"
#include "examples/nearest_points/segseg.hpp"
#include "examples/nearest_points/lineline.hpp"

#define REGISTER_EXAMPLE(name) \
    ctx.RegisterExample<name##Example>(ctx, #name);

void registerExamples(Context& ctx) {
    REGISTER_EXAMPLE(SphereNearestPoint);
    REGISTER_EXAMPLE(CapsuleNearestPoint);
    REGISTER_EXAMPLE(SegmentNearestPoint);
    REGISTER_EXAMPLE(OBBNearestPoint);
    REGISTER_EXAMPLE(TetrahedronNearestPoint);
    REGISTER_EXAMPLE(TriangleNearestPoint);
    REGISTER_EXAMPLE(SegSegNearestPoint);
    REGISTER_EXAMPLE(LineLineNearestPoint);
    REGISTER_EXAMPLE(RenderTest);
}

int main(int argc, char** argv) {
    Context::Init();
    auto& inst = Context::GetInst();
    inst.Initialize();
    registerExamples(inst);
    inst.Update();
    inst.Shutdown();
    Context::Destroy();

    return 0;
}