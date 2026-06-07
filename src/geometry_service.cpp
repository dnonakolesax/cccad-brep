#include "cccad/geometry/geometry_service.hpp"
#include "cccad/geometry/boolean_builder.hpp"
#include "cccad/geometry/edge_modifier_builder.hpp"
#include "cccad/geometry/extrude_builder.hpp"
#include "cccad/geometry/hole_builder.hpp"
#include "cccad/geometry/pattern_builder.hpp"

#include <exception>
#include <stdexcept>
#include <string>
#include <vector>

namespace cccad::geometry {

namespace {

void add_diagnostic(cccad::geometry::v1::BuildFeatureResponse* response,
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

void add_diagnostic(cccad::geometry::v1::BuildFeatureResponse* response,
                    const cccad::geometry::v1::BuildHoleRequest& request,
                    const std::string& code,
                    const std::string& severity,
                    const std::string& message) {
  auto* d = response->add_diagnostics();
  d->set_code(code);
  d->set_severity(severity);
  d->set_message(message);
  d->set_sketch_id(request.sketch_id());
  d->set_feature_id(request.feature_id());
  d->set_body_id(request.parameters().target_body_id());
}

void add_diagnostic(cccad::geometry::v1::BuildFeatureResponse* response,
                    const cccad::geometry::v1::BuildFilletRequest& request,
                    const std::string& code,
                    const std::string& severity,
                    const std::string& message) {
  auto* d = response->add_diagnostics();
  d->set_code(code);
  d->set_severity(severity);
  d->set_message(message);
  d->set_feature_id(request.feature_id());
  d->set_body_id(request.target_body_id());
}

void add_diagnostic(cccad::geometry::v1::BuildFeatureResponse* response,
                    const cccad::geometry::v1::BuildChamferRequest& request,
                    const std::string& code,
                    const std::string& severity,
                    const std::string& message) {
  auto* d = response->add_diagnostics();
  d->set_code(code);
  d->set_severity(severity);
  d->set_message(message);
  d->set_feature_id(request.feature_id());
  d->set_body_id(request.target_body_id());
}

void add_diagnostic(cccad::geometry::v1::BuildFeatureResponse* response,
                    const cccad::geometry::v1::BuildBooleanRequest& request,
                    const std::string& code,
                    const std::string& severity,
                    const std::string& message) {
  auto* d = response->add_diagnostics();
  d->set_code(code);
  d->set_severity(severity);
  d->set_message(message);
  d->set_feature_id(request.feature_id());
  d->set_body_id(request.target_body_id());
}

void add_diagnostic(cccad::geometry::v1::BuildFeatureResponse* response,
                    const cccad::geometry::v1::BuildPatternRequest& request,
                    const std::string& code,
                    const std::string& severity,
                    const std::string& message) {
  auto* d = response->add_diagnostics();
  d->set_code(code);
  d->set_severity(severity);
  d->set_message(message);
  d->set_feature_id(request.feature_id());
}

const cccad::geometry::v1::BodyInput& find_body_by_id(
    const google::protobuf::RepeatedPtrField<cccad::geometry::v1::BodyInput>& bodies,
    const std::string& body_id,
    const std::string& role) {
  if (body_id.empty()) {
    throw std::runtime_error(role + " body_id is required");
  }

  for (const auto& body : bodies) {
    if (body.body_id() == body_id) {
      return body;
    }
  }

  throw std::runtime_error(role + " body was not found in existing_bodies: " + body_id);
}

const cccad::geometry::v1::BodyInput& find_target_body(
    const cccad::geometry::v1::BuildHoleRequest& request) {
  const std::string& target_body_id = request.parameters().target_body_id();
  if (target_body_id.empty()) {
    throw std::runtime_error("hole target_body_id is required");
  }

  for (const auto& body : request.existing_bodies()) {
    if (body.body_id() == target_body_id) {
      return body;
    }
  }

  throw std::runtime_error("target body was not found in existing_bodies: " + target_body_id);
}

const cccad::geometry::v1::BodyInput& find_target_body(
    const cccad::geometry::v1::BuildFilletRequest& request) {
  const std::string& target_body_id = request.target_body_id();
  if (target_body_id.empty()) {
    throw std::runtime_error("fillet target_body_id is required");
  }

  for (const auto& body : request.existing_bodies()) {
    if (body.body_id() == target_body_id) {
      return body;
    }
  }

  throw std::runtime_error("target body was not found in existing_bodies: " + target_body_id);
}

const cccad::geometry::v1::BodyInput& find_target_body(
    const cccad::geometry::v1::BuildChamferRequest& request) {
  const std::string& target_body_id = request.target_body_id();
  if (target_body_id.empty()) {
    throw std::runtime_error("chamfer target_body_id is required");
  }

  for (const auto& body : request.existing_bodies()) {
    if (body.body_id() == target_body_id) {
      return body;
    }
  }

  throw std::runtime_error("target body was not found in existing_bodies: " + target_body_id);
}

const cccad::geometry::v1::BodyInput& find_target_body(
    const cccad::geometry::v1::BuildBooleanRequest& request) {
  return find_body_by_id(request.existing_bodies(), request.target_body_id(), "boolean target");
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
    cccad::geometry::v1::BuildFeatureResponse* response) {
  response->set_request_id(request->context().request_id());

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

grpc::Status GeometryKernelServiceImpl::BuildHole(
    grpc::ServerContext*,
    const cccad::geometry::v1::BuildHoleRequest* request,
    cccad::geometry::v1::BuildFeatureResponse* response) {
  response->set_request_id(request->context().request_id());

  try {
    const auto& target_body = find_target_body(*request);
    TopoDS_Shape target_shape = artifact_writer_.read_brep_artifact(target_body.brep());
    TopoDS_Shape shape = build_hole_shape(*request, target_shape);
    const auto artifacts = artifact_writer_.write_requested_artifacts(*request, shape);

    auto* body = response->add_bodies();
    body->set_body_id(request->parameters().target_body_id());
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
    add_diagnostic(response, *request, "HOLE_FAILED", "error", ex.what());
    return grpc::Status::OK;
  }
}

grpc::Status GeometryKernelServiceImpl::BuildFillet(
    grpc::ServerContext*,
    const cccad::geometry::v1::BuildFilletRequest* request,
    cccad::geometry::v1::BuildFeatureResponse* response) {
  response->set_request_id(request->context().request_id());

  try {
    const auto& target_body = find_target_body(*request);
    TopoDS_Shape target_shape = artifact_writer_.read_brep_artifact(target_body.brep());
    TopoDS_Shape shape = build_fillet_shape(*request, target_shape);
    const auto artifacts = artifact_writer_.write_requested_artifacts(*request, shape);

    auto* body = response->add_bodies();
    body->set_body_id(request->target_body_id());
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
    add_diagnostic(response, *request, "FILLET_FAILED", "error", ex.what());
    return grpc::Status::OK;
  }
}

grpc::Status GeometryKernelServiceImpl::BuildChamfer(
    grpc::ServerContext*,
    const cccad::geometry::v1::BuildChamferRequest* request,
    cccad::geometry::v1::BuildFeatureResponse* response) {
  response->set_request_id(request->context().request_id());

  try {
    const auto& target_body = find_target_body(*request);
    TopoDS_Shape target_shape = artifact_writer_.read_brep_artifact(target_body.brep());
    TopoDS_Shape shape = build_chamfer_shape(*request, target_shape);
    const auto artifacts = artifact_writer_.write_requested_artifacts(*request, shape);

    auto* body = response->add_bodies();
    body->set_body_id(request->target_body_id());
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
    add_diagnostic(response, *request, "CHAMFER_FAILED", "error", ex.what());
    return grpc::Status::OK;
  }
}

grpc::Status GeometryKernelServiceImpl::BuildBoolean(
    grpc::ServerContext*,
    const cccad::geometry::v1::BuildBooleanRequest* request,
    cccad::geometry::v1::BuildFeatureResponse* response) {
  response->set_request_id(request->context().request_id());

  try {
    const auto& target_body = find_target_body(*request);
    TopoDS_Shape target_shape = artifact_writer_.read_brep_artifact(target_body.brep());

    std::vector<TopoDS_Shape> tool_shapes;
    tool_shapes.reserve(static_cast<std::size_t>(request->tool_body_ids_size()));
    for (const auto& tool_body_id : request->tool_body_ids()) {
      if (tool_body_id == request->target_body_id()) {
        throw std::runtime_error("boolean tool body cannot be the target body: " + tool_body_id);
      }

      const auto& tool_body = find_body_by_id(request->existing_bodies(), tool_body_id, "boolean tool");
      tool_shapes.push_back(artifact_writer_.read_brep_artifact(tool_body.brep()));
    }

    TopoDS_Shape shape = build_boolean_shape(*request, target_shape, tool_shapes);
    const auto artifacts = artifact_writer_.write_requested_artifacts(*request, shape);

    auto* body = response->add_bodies();
    body->set_body_id(request->target_body_id());
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
    add_diagnostic(response, *request, "BOOLEAN_FAILED", "error", ex.what());
    return grpc::Status::OK;
  }
}

grpc::Status GeometryKernelServiceImpl::BuildPattern(
    grpc::ServerContext*,
    const cccad::geometry::v1::BuildPatternRequest* request,
    cccad::geometry::v1::BuildFeatureResponse* response) {
  response->set_request_id(request->context().request_id());

  try {
    std::vector<TopoDS_Shape> source_shapes;
    source_shapes.reserve(static_cast<std::size_t>(request->source_body_ids_size()));
    for (const auto& source_body_id : request->source_body_ids()) {
      const auto& source_body = find_body_by_id(request->existing_bodies(), source_body_id, "pattern source");
      source_shapes.push_back(artifact_writer_.read_brep_artifact(source_body.brep()));
    }

    TopoDS_Shape shape = build_pattern_shape(*request, source_shapes);
    const auto artifacts = artifact_writer_.write_requested_artifacts(*request, shape);

    auto* body = response->add_bodies();
    body->set_body_id(request->feature_id().empty() ? "body-pattern" : ("body-" + request->feature_id()));
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
    add_diagnostic(response, *request, "PATTERN_FAILED", "error", ex.what());
    return grpc::Status::OK;
  }
}

} // namespace cccad::geometry
