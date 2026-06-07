#pragma once

#include <memory>
#include <string>

#include <grpcpp/grpcpp.h>
#include "geometry_kernel.grpc.pb.h"
#include "cccad/geometry/artifact_writer.hpp"

namespace cccad::geometry {

class GeometryKernelServiceImpl final : public cccad::geometry::v1::GeometrySolverService::Service {
public:
  explicit GeometryKernelServiceImpl(ArtifactWriter artifact_writer);

  grpc::Status Health(grpc::ServerContext* context,
                      const cccad::geometry::v1::HealthRequest* request,
                      cccad::geometry::v1::HealthResponse* response) override;

  grpc::Status BuildExtrude(grpc::ServerContext* context,
                            const cccad::geometry::v1::BuildExtrudeRequest* request,
                            cccad::geometry::v1::BuildFeatureResponse* response) override;

  grpc::Status BuildHole(grpc::ServerContext* context,
                         const cccad::geometry::v1::BuildHoleRequest* request,
                         cccad::geometry::v1::BuildFeatureResponse* response) override;

  grpc::Status BuildFillet(grpc::ServerContext* context,
                           const cccad::geometry::v1::BuildFilletRequest* request,
                           cccad::geometry::v1::BuildFeatureResponse* response) override;

  grpc::Status BuildChamfer(grpc::ServerContext* context,
                            const cccad::geometry::v1::BuildChamferRequest* request,
                            cccad::geometry::v1::BuildFeatureResponse* response) override;

  grpc::Status BuildBoolean(grpc::ServerContext* context,
                            const cccad::geometry::v1::BuildBooleanRequest* request,
                            cccad::geometry::v1::BuildFeatureResponse* response) override;

  grpc::Status BuildPattern(grpc::ServerContext* context,
                            const cccad::geometry::v1::BuildPatternRequest* request,
                            cccad::geometry::v1::BuildFeatureResponse* response) override;

private:
  ArtifactWriter artifact_writer_;
};

} // namespace cccad::geometry
