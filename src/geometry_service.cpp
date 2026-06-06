#include "cccad/geometry/geometry_service.hpp"
#include "cccad/geometry/extrude_builder.hpp"

#include <exception>
#include <string>

namespace cccad::geometry {

namespace {

void add_diagnostic(cccad::geometry::v1::BuildExtrudeResponse* response,
                    const cccad::geometry::v1::BuildExtrudeRequest& request,
                    const std::string& code,
                    const std::string& severity,
                    const std::string& message) {
  auto* d = response->add_diagnostics();
  d->set_code(code);
  d->set_severity(severity);
  d->set_message(message);
  d->set_sketch_id(request.sketch_id());
  d->set_profile_id(request.profile().profile_id());
  d->set_feature_id(request.feature_id());
}

} // namespace

GeometryKernelServiceImpl::GeometryKernelServiceImpl(ArtifactWriter artifact_writer)
    : artifact_writer_(std::move(artifact_writer)) {}

grpc::Status GeometryKernelServiceImpl::Health(
    grpc::ServerContext*,
    const cccad::geometry::v1::HealthRequest*,
    cccad::geometry::v1::HealthResponse* response) {
  response->set_status("ok");
  response->set_kernel_name("opencascade");
  response->set_kernel_version("system");
  return grpc::Status::OK;
}

grpc::Status GeometryKernelServiceImpl::BuildExtrude(
    grpc::ServerContext*,
    const cccad::geometry::v1::BuildExtrudeRequest* request,
    cccad::geometry::v1::BuildExtrudeResponse* response) {
  response->set_request_id(request->request_id());
  response->set_part_id(request->part_id());
  response->set_feature_id(request->feature_id());

  try {
    TopoDS_Shape shape = build_extrude_shape(*request);
    const auto artifacts = artifact_writer_.write_requested_artifacts(*request, shape);

    auto* body = response->add_bodies();
    body->set_body_id("body-" + request->feature_id());
    body->set_created_by_feature_id(request->feature_id());

    for (const auto& a : artifacts) {
      auto* top_artifact = response->add_artifacts();
      top_artifact->set_kind(a.kind);
      top_artifact->set_storage_key(a.storage_key);
      top_artifact->set_content_type(a.content_type);
      top_artifact->set_size_bytes(static_cast<std::int64_t>(a.size_bytes));

      auto* body_artifact = body->add_artifacts();
      body_artifact->CopyFrom(*top_artifact);
    }

    response->set_success(true);
    return grpc::Status::OK;
  } catch (const std::exception& ex) {
    response->set_success(false);
    add_diagnostic(response, *request, "EXTRUDE_FAILED", "error", ex.what());
    return grpc::Status::OK;
  }
}

} // namespace cccad::geometry
