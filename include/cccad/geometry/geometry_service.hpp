#pragma once

#include <memory>
#include <string>

#include <grpcpp/grpcpp.h>
#include "geometry_kernel.grpc.pb.h"
#include "cccad/geometry/artifact_writer.hpp"

namespace cccad::geometry {

class GeometryKernelServiceImpl final : public cccad::geometry::v1::GeometryKernelService::Service {
public:
  explicit GeometryKernelServiceImpl(ArtifactWriter artifact_writer);

  grpc::Status Health(grpc::ServerContext* context,
                      const cccad::geometry::v1::HealthRequest* request,
                      cccad::geometry::v1::HealthResponse* response) override;

  grpc::Status BuildExtrude(grpc::ServerContext* context,
                            const cccad::geometry::v1::BuildExtrudeRequest* request,
                            cccad::geometry::v1::BuildExtrudeResponse* response) override;

private:
  ArtifactWriter artifact_writer_;
};

} // namespace cccad::geometry
