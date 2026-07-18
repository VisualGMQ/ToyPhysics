#pragma once

namespace toy_physics {

#ifndef TOY_PHYSICS_CHECK
#ifdef TOY_PHYSICS_DEBUG
#define TOY_PHYSICS_CHECK 1
#else
#define TOY_PHYSICS_CHECK 0
#endif
#endif

#ifdef TOY_PHYSICS_DOUBLE
using real = double;
#else
using real = float;
#endif

}
