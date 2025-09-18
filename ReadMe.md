# ToyPhysics

For learning game physics engine

## How To Build

need `vcpkg`:

```bash
cmake -S . -B cmake-build -DCMAKE_TOOLCHAIN_FILE="$env{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake"
cmake --build cmake-build
```