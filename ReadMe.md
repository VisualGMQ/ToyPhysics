# ToyPhysics

For learning game physics engine

## How To Build

build using cmake

```bash
cmake -S . -B cmake-build
cmake --build cmake-build
```

it will auto download dependencies.


or build to web using emscripten:

```bash
emcmake cmake -S . -B emcmake-build
cmake --build emcmake-build --target sandbox
```

will output `sandbox.html`.

## How to generate doc

Using [Doxygen](https://www.doxygen.nl/index.html).

NOTE: mathjax render need connect to net

```bash
doxygen doc/Doxyfile
```

it will generate to doxygen-gen.

## Code Read Note

`sandbox/` is wrote in AI completely. DON'T read them.