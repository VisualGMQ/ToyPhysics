#pragma once
#include <cstdint>

namespace toy_physics {

enum class StorageID: uint32_t {};

// some alias to make ID more clear
using ShapeID = StorageID;
using DynamicRigidBodyID = StorageID;
using StaticRigidBodyID = StorageID;

enum class TightPoolID: uint32_t {};

}
