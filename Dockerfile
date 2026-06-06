# syntax=docker/dockerfile:1.7

FROM ubuntu:24.04 AS builder

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    ninja-build \
    pkg-config \
    git \
    ca-certificates \
    protobuf-compiler \
    protobuf-compiler-grpc \
    libprotobuf-dev \
    libgrpc++-dev \
    libopencascade-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

COPY CMakeLists.txt ./
COPY proto ./proto
COPY include ./include
COPY src ./src

RUN cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
RUN cmake --build build --target cccad-geometry-service -j"$(nproc)"

FROM ubuntu:24.04 AS runtime

ENV DEBIAN_FRONTEND=noninteractive

# Runtime uses dev packages intentionally for MVP reliability: Ubuntu package names for exact
# OpenCascade runtime libraries are easy to mismatch between minor images.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    libprotobuf-dev \
    libgrpc++-dev \
    libopencascade-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

COPY --from=builder /app/build/cccad-geometry-service /app/cccad-geometry-service

ENV GEOMETRY_SERVICE_HOST=0.0.0.0
ENV GEOMETRY_SERVICE_PORT=50051
ENV GEOMETRY_STORAGE_ROOT=/data/geometry

RUN mkdir -p /data/geometry

EXPOSE 50051

CMD ["/app/cccad-geometry-service"]
