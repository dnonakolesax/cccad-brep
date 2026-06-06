# cccAD Geometry Kernel Service

Minimal C++/OpenCascade geometry-kernel-service for cccAD.

## What is implemented

- gRPC service skeleton
- `Health`
- `BuildExtrude`
- OpenCascade-based extrusion
- Supported profiles:
  - ordered closed line loop
  - single full circle profile
- Supported extrude directions:
  - `forward`
  - `backward`
  - `symmetric`
- Supported operation:
  - `new_body`
- Artifact writing:
  - `.brep`
  - `.mesh.json`

## Not implemented yet

- `join` / `cut` booleans
- arcs in profile loops
- inner loops / holes inside a profile
- GLB export
- STEP/STL export
- topology summary output
- `RebuildPart`
- `GetFacePlane`

These are intentionally left as next steps. The current service is the minimal core needed to move from frontend-only extrude to authoritative backend geometry building.

## Build locally

Requires Ubuntu packages:

```bash
sudo apt-get update
sudo apt-get install -y \
  build-essential cmake ninja-build protobuf-compiler protobuf-compiler-grpc \
  libprotobuf-dev libgrpc++-dev libopencascade-dev
```

Build:

```bash
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build --target cccad-geometry-service -j"$(nproc)"
```

Run:

```bash
GEOMETRY_STORAGE_ROOT=/tmp/cccad-geometry ./build/cccad-geometry-service
```

## Docker

```bash
docker compose up -d --build
```

Service listens on:

```text
0.0.0.0:50051
```

Artifacts are written under:

```text
/data/geometry
```

## BuildExtrude request notes

The Go backend should send a normalized sketch plane:

```text
origin, x_axis, y_axis, normal
normal = cross(x_axis, y_axis)
```

The sketch profile must already be an ordered closed profile. The service deliberately does not solve sketch constraints or search for profiles inside raw sketch entities.

For MVP, send:

```text
parameters.operation = "new_body"
output.write_brep = true
output.write_mesh_json = true
```

The service returns `success=false` with structured diagnostics if the profile/plane/depth is invalid or unsupported.
