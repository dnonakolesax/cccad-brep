# syntax=docker/dockerfile:1.7

FROM debian:13-slim as builder

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get clean \
    && apt-get update -o Acquire::Retries=5 \
    && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        ninja-build \
        pkg-config \
        git \
        ca-certificates \
        protobuf-compiler \
        libprotobuf-dev \
        libgrpc++-dev \
        protobuf-compiler-grpc \
        libocct-foundation-dev \
        libocct-modeling-data-dev \
        libocct-modeling-algorithms-dev \
        libocct-data-exchange-dev \
        libocct-ocaf-dev \
        nlohmann-json3-dev \
        libspdlog-dev \
        libfmt-dev

WORKDIR /app

COPY CMakeLists.txt ./
COPY proto ./proto
COPY include ./include
COPY src ./src

RUN cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
RUN cmake --build build --target cccad-geometry-service -j"$(nproc)"

FROM debian:13-slim AS runtime

ENV DEBIAN_FRONTEND=noninteractive

# Runtime uses dev packages intentionally for MVP reliability: Ubuntu package names for exact
# OpenCascade runtime libraries are easy to mismatch between minor images.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    libprotobuf-dev \
    libgrpc++-dev \
    libocct-foundation-dev \
    libocct-modeling-data-dev \
    libocct-modeling-algorithms-dev \
    libocct-data-exchange-dev \
    libocct-ocaf-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

COPY --from=builder /app/build/cccad-geometry-service /app/cccad-geometry-service

ENV GEOMETRY_SERVICE_HOST=0.0.0.0
ENV GEOMETRY_SERVICE_PORT=50051
ENV GEOMETRY_STORAGE_ROOT=/data/geometry

RUN mkdir -p /data/geometry

EXPOSE 50051

CMD ["/app/cccad-geometry-service"]
