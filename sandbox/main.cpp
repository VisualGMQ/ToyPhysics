#include "context.hpp"
#include "examples/render_test/render_test.hpp"
#include "examples/nearest_points/sphere.hpp"
#include "examples/nearest_points/capsule.hpp"
#include "examples/nearest_points/cylinder.hpp"
#include "examples/nearest_points/segment.hpp"
#include "examples/nearest_points/obb.hpp"
#include "examples/nearest_points/tetrahedron.hpp"
#include "examples/nearest_points/triangle.hpp"
#include "examples/nearest_points/segseg.hpp"
#include "examples/nearest_points/lineline.hpp"
#include "examples/gjk/gjk_tetrahedrons.hpp"
#include "examples/mtd/mtd_tetrahedrons.hpp"
#include "examples/raycast/sphere.hpp"
#include "examples/raycast/aabb.hpp"
#include "examples/raycast/obb.hpp"
#include "examples/raycast/plane.hpp"
#include "examples/raycast/triangle.hpp"
#include "examples/raycast/capsule.hpp"
#include "examples/raycast/cylinder.hpp"
#include "examples/raycast/convex.hpp"
#include "examples/sweep/sphere_obb.hpp"
#include "examples/sweep/sphere_sphere.hpp"
#include "examples/sweep/sphere_capsule.hpp"
#include "examples/sweep/sphere_cylinder.hpp"
#include "examples/sweep/sphere_triangle.hpp"
#include "examples/sweep/sphere_convex.hpp"
#include "examples/sweep/capsule_capsule.hpp"
#include "examples/sweep/capsule_obb.hpp"
#include "examples/sweep/capsule_cylinder.hpp"
#include "examples/sweep/capsule_convex.hpp"
#include "examples/sweep/cylinder_cylinder.hpp"
#include "examples/sweep/cylinder_convex.hpp"
#include "examples/sweep/convex_convex.hpp"

#ifdef TOY_PHYSICS_PLATFORM_WEB
#include "emscripten.h"
#endif

#define REGISTER_EXAMPLE(name, label) \
    ctx.RegisterExample<name##Example>(ctx, label);

void registerExamples(Context& ctx) {
    REGISTER_EXAMPLE(SphereNearestPoint, "NearestPoint/Sphere");
    REGISTER_EXAMPLE(CapsuleNearestPoint, "NearestPoint/Capsule");
    REGISTER_EXAMPLE(CylinderNearestPoint, "NearestPoint/Cylinder");
    REGISTER_EXAMPLE(SegmentNearestPoint, "NearestPoint/Segment");
    REGISTER_EXAMPLE(OBBNearestPoint, "NearestPoint/OBB");
    REGISTER_EXAMPLE(TetrahedronNearestPoint, "NearestPoint/Tetrahedron");
    REGISTER_EXAMPLE(TriangleNearestPoint, "NearestPoint/Triangle");
    REGISTER_EXAMPLE(SegSegNearestPoint, "NearestPoint/SegSeg");
    REGISTER_EXAMPLE(LineLineNearestPoint, "NearestPoint/LineLine");
    REGISTER_EXAMPLE(GjkTetrahedrons, "NearestPoint/GJK Tetrahedrons");
    REGISTER_EXAMPLE(MtdTetrahedrons, "MTD/Tetrahedrons");
    REGISTER_EXAMPLE(SphereRaycast, "Raycast/Sphere");
    REGISTER_EXAMPLE(AABBRaycast, "Raycast/AABB");
    REGISTER_EXAMPLE(OBBRaycast, "Raycast/OBB");
    REGISTER_EXAMPLE(PlaneRaycast, "Raycast/Plane");
    REGISTER_EXAMPLE(TriangleRaycast, "Raycast/Triangle");
    REGISTER_EXAMPLE(CapsuleRaycast, "Raycast/Capsule");
    REGISTER_EXAMPLE(CylinderRaycast, "Raycast/Cylinder");
    REGISTER_EXAMPLE(ConvexRaycast, "Raycast/Convex");
    REGISTER_EXAMPLE(SphereOBBSweep, "Sweep/Sphere OBB");
    REGISTER_EXAMPLE(SphereSphereSweep, "Sweep/Sphere Sphere");
    REGISTER_EXAMPLE(SphereCapsuleSweep, "Sweep/Sphere Capsule");
    REGISTER_EXAMPLE(SphereCylinderSweep, "Sweep/Sphere Cylinder");
    REGISTER_EXAMPLE(SphereTriangleSweep, "Sweep/Sphere Triangle");
    REGISTER_EXAMPLE(SphereConvexSweep, "Sweep/Sphere Convex");
    REGISTER_EXAMPLE(CapsuleCapsuleSweep, "Sweep/Capsule Capsule");
    REGISTER_EXAMPLE(CapsuleOBBSweep, "Sweep/Capsule OBB");
    REGISTER_EXAMPLE(CapsuleCylinderSweep, "Sweep/Capsule Cylinder");
    REGISTER_EXAMPLE(CapsuleConvexSweep, "Sweep/Capsule Convex");
    REGISTER_EXAMPLE(CylinderCylinderSweep, "Sweep/Cylinder Cylinder");
    REGISTER_EXAMPLE(CylinderConvexSweep, "Sweep/Cylinder Convex");
    REGISTER_EXAMPLE(ConvexConvexSweep, "Sweep/Convex Convex");
    REGISTER_EXAMPLE(RenderTest, "RenderTest");
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