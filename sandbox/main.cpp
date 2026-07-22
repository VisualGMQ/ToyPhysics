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
#include "examples/gjk/gjk_tetrahedrons.hpp"
#include "examples/mtd/mtd_tetrahedrons.hpp"

#ifdef TOY_PHYSICS_PLATFORM_WEB
#include "emscripten.h"
#endif

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
    REGISTER_EXAMPLE(GjkTetrahedrons);
    REGISTER_EXAMPLE(MtdTetrahedrons);
    REGISTER_EXAMPLE(RenderTest);
}

#ifdef TOY_PHYSICS_PLATFORM_WEB
void MainLoop() {
    Context::GetInst().Update();
}
#endif

int main(int argc, char** argv) {
    Context::Init();
    auto& inst = Context::GetInst();
    inst.Initialize();
    registerExamples(inst);

#ifdef TOY_PHYSICS_PLATFORM_WEB
    emscripten_set_main_loop(MainLoop, 0, 1);
#else
    inst.Update();
#endif

    inst.Shutdown();
    Context::Destroy();

    return 0;
}