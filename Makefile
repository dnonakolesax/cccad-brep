BUILD_DIR ?= build
BUILD_TYPE ?= Release
GENERATOR ?= Ninja
TARGET ?= cccad-geometry-service
JOBS ?= $(shell nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)
GEOMETRY_STORAGE_ROOT ?= /tmp/cccad-geometry

CMAKE_CONFIGURE_FLAGS ?=
CMAKE_BUILD_FLAGS ?=

.PHONY: help configure build all debug release run clean distclean rebuild test docker-build docker-up docker-down

help:
	@printf '%s\n' 'Available targets:'
	@printf '  %-14s %s\n' 'configure' 'Configure CMake build directory'
	@printf '  %-14s %s\n' 'build/all' 'Build the geometry service'
	@printf '  %-14s %s\n' 'debug' 'Build with BUILD_TYPE=Debug'
	@printf '  %-14s %s\n' 'release' 'Build with BUILD_TYPE=Release'
	@printf '  %-14s %s\n' 'run' 'Build and run the service locally'
	@printf '  %-14s %s\n' 'test' 'Run CTest from the build directory'
	@printf '  %-14s %s\n' 'clean' 'Clean compiled artifacts in the build directory'
	@printf '  %-14s %s\n' 'distclean' 'Remove the build directory'
	@printf '  %-14s %s\n' 'rebuild' 'Remove build directory, configure, and build'
	@printf '  %-14s %s\n' 'docker-build' 'Build the Docker image via Compose'
	@printf '  %-14s %s\n' 'docker-up' 'Start the service via Compose'
	@printf '  %-14s %s\n' 'docker-down' 'Stop the Compose service'
	@printf '\n%s\n' 'Variables:'
	@printf '  %-14s %s\n' 'BUILD_DIR' 'Build directory (default: build)'
	@printf '  %-14s %s\n' 'BUILD_TYPE' 'CMake build type (default: Release)'
	@printf '  %-14s %s\n' 'GENERATOR' 'CMake generator (default: Ninja)'
	@printf '  %-14s %s\n' 'TARGET' 'Build target (default: cccad-geometry-service)'
	@printf '  %-14s %s\n' 'JOBS' 'Parallel build jobs (default: CPU count)'

configure:
	cmake -S . -B "$(BUILD_DIR)" -G "$(GENERATOR)" -DCMAKE_BUILD_TYPE="$(BUILD_TYPE)" $(CMAKE_CONFIGURE_FLAGS)

build all: configure
	cmake --build "$(BUILD_DIR)" --target "$(TARGET)" -j"$(JOBS)" $(CMAKE_BUILD_FLAGS)

debug:
	$(MAKE) BUILD_TYPE=Debug build

release:
	$(MAKE) BUILD_TYPE=Release build

run: build
	GEOMETRY_STORAGE_ROOT="$(GEOMETRY_STORAGE_ROOT)" "$(BUILD_DIR)/$(TARGET)"

test: configure
	ctest --test-dir "$(BUILD_DIR)" --output-on-failure

clean:
	cmake --build "$(BUILD_DIR)" --target clean

distclean:
	rm -rf "$(BUILD_DIR)"

rebuild: distclean build

docker-build:
	docker compose build

docker-up:
	docker compose up -d --build

docker-down:
	docker compose down
