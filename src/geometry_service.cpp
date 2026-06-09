#include "cccad/geometry/geometry_service.hpp"
#include "cccad/geometry/boolean_builder.hpp"
#include "cccad/geometry/edge_modifier_builder.hpp"
#include "cccad/geometry/extrude_builder.hpp"
#include "cccad/geometry/hole_builder.hpp"
#include "cccad/geometry/pattern_builder.hpp"
#include "cccad/geometry/topology_builder.hpp"

#include <exception>
#include <algorithm>
#include <map>
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

void add_diagnostic(cccad::geometry::v1::RebuildPartResponse* response,
                    const cccad::geometry::v1::KernelFeature& feature,
                    const std::string& code,
                    const std::string& severity,
                    const std::string& message) {
  auto* d = response->add_diagnostics();
  d->set_code(code);
  d->set_severity(severity);
  d->set_message(message);
  d->set_feature_id(feature.feature_id());
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

struct BodyState {
  TopoDS_Shape shape;
  std::string created_by_feature_id;
};

const cccad::geometry::v1::KernelSketch& find_sketch(
    const cccad::geometry::v1::RebuildPartRequest& request,
    const std::string& sketch_id) {
  if (sketch_id.empty()) {
    throw std::runtime_error("sketch_id is required");
  }

  for (const auto& sketch : request.sketches()) {
    if (sketch.sketch_id() == sketch_id) {
      return sketch;
    }
  }

  throw std::runtime_error("sketch was not found: " + sketch_id);
}

const cccad::geometry::v1::SketchProfile& find_profile(
    const cccad::geometry::v1::KernelSketch& sketch,
    const std::string& profile_id) {
  if (profile_id.empty() && sketch.profiles_size() == 1) {
    return sketch.profiles(0);
  }

  for (const auto& profile : sketch.profiles()) {
    if (profile.profile_id() == profile_id) {
      return profile;
    }
  }

  throw std::runtime_error("profile was not found in sketch: " + profile_id);
}

const BodyState& find_body_state(const std::map<std::string, BodyState>& bodies,
                                 const std::string& body_id,
                                 const std::string& role) {
  if (body_id.empty()) {
    throw std::runtime_error(role + " body_id is required");
  }

  const auto it = bodies.find(body_id);
  if (it == bodies.end()) {
    throw std::runtime_error(role + " body was not found: " + body_id);
  }

  return it->second;
}

std::vector<TopoDS_Shape> collect_pattern_source_shapes(
    const cccad::geometry::v1::PatternFeature& pattern,
    const std::map<std::string, BodyState>& bodies,
    const std::map<std::string, std::vector<std::string>>& body_ids_by_feature_id) {
  std::vector<TopoDS_Shape> source_shapes;

  for (const auto& body_id : pattern.source_body_ids()) {
    source_shapes.push_back(find_body_state(bodies, body_id, "pattern source").shape);
  }

  for (const auto& feature_id : pattern.source_feature_ids()) {
    const auto it = body_ids_by_feature_id.find(feature_id);
    if (it == body_ids_by_feature_id.end()) {
      throw std::runtime_error("pattern source feature was not found: " + feature_id);
    }

    for (const auto& body_id : it->second) {
      source_shapes.push_back(find_body_state(bodies, body_id, "pattern source").shape);
    }
  }

  return source_shapes;
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

    if (request->output().return_topology()) {
      add_shape_topology(body->body_id(), shape, response->mutable_topology());
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

    if (request->output().return_topology()) {
      add_shape_topology(body->body_id(), shape, response->mutable_topology());
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

    if (request->output().return_topology()) {
      add_shape_topology(body->body_id(), shape, response->mutable_topology());
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

    if (request->output().return_topology()) {
      add_shape_topology(body->body_id(), shape, response->mutable_topology());
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

    if (request->output().return_topology()) {
      add_shape_topology(body->body_id(), shape, response->mutable_topology());
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

    if (request->output().return_topology()) {
      add_shape_topology(body->body_id(), shape, response->mutable_topology());
    }

    response->set_success(true);
    return grpc::Status::OK;
  } catch (const std::exception& ex) {
    response->set_success(false);
    add_diagnostic(response, *request, "PATTERN_FAILED", "error", ex.what());
    return grpc::Status::OK;
  }
}

grpc::Status GeometryKernelServiceImpl::RebuildPart(
    grpc::ServerContext*,
    const cccad::geometry::v1::RebuildPartRequest* request,
    cccad::geometry::v1::RebuildPartResponse* response) {
  response->set_request_id(request->context().request_id());
  response->set_document_version(request->context().document_version());

  std::map<std::string, BodyState> bodies;
  std::map<std::string, std::vector<std::string>> body_ids_by_feature_id;

  try {
    for (const auto& body : request->existing_bodies()) {
      bodies[body.body_id()] = BodyState{
          .shape = artifact_writer_.read_brep_artifact(body.brep()),
          .created_by_feature_id = "",
      };
    }

    std::vector<const cccad::geometry::v1::KernelFeature*> features;
    features.reserve(static_cast<std::size_t>(request->features_size()));
    for (const auto& feature : request->features()) {
      features.push_back(&feature);
    }

    std::stable_sort(features.begin(), features.end(),
                     [](const auto* lhs, const auto* rhs) {
                       return lhs->order_index() < rhs->order_index();
                     });

    for (const auto* feature : features) {
      if (feature->suppressed()) {
        continue;
      }

      try {
        switch (feature->feature_case()) {
          case cccad::geometry::v1::KernelFeature::kExtrude: {
            const auto& extrude = feature->extrude();
            const auto& sketch = find_sketch(*request, extrude.sketch_id());
            const auto& profile = find_profile(sketch, extrude.profile_id());

            cccad::geometry::v1::BuildExtrudeRequest build_request;
            build_request.mutable_context()->CopyFrom(request->context());
            build_request.set_feature_id(feature->feature_id());
            build_request.set_sketch_id(sketch.sketch_id());
            build_request.mutable_sketch_plane()->CopyFrom(sketch.plane());
            build_request.mutable_profile()->CopyFrom(profile);
            build_request.mutable_parameters()->CopyFrom(extrude.parameters());
            build_request.mutable_output()->CopyFrom(request->output());

            const std::string body_id = feature->feature_id().empty() ? "body" : ("body-" + feature->feature_id());
            bodies[body_id] = BodyState{
                .shape = build_extrude_shape(build_request),
                .created_by_feature_id = feature->feature_id(),
            };
            body_ids_by_feature_id[feature->feature_id()] = {body_id};
            break;
          }
          case cccad::geometry::v1::KernelFeature::kHole: {
            const auto& hole = feature->hole();
            const auto& sketch = find_sketch(*request, hole.sketch_id());
            const std::string& body_id = hole.parameters().target_body_id();
            const auto& target = find_body_state(bodies, body_id, "hole target");

            cccad::geometry::v1::BuildHoleRequest build_request;
            build_request.mutable_context()->CopyFrom(request->context());
            build_request.set_feature_id(feature->feature_id());
            build_request.set_sketch_id(sketch.sketch_id());
            build_request.mutable_sketch_plane()->CopyFrom(sketch.plane());
            build_request.mutable_center()->CopyFrom(hole.center());
            build_request.mutable_parameters()->CopyFrom(hole.parameters());
            build_request.mutable_output()->CopyFrom(request->output());

            bodies[body_id] = BodyState{
                .shape = build_hole_shape(build_request, target.shape),
                .created_by_feature_id = feature->feature_id(),
            };
            body_ids_by_feature_id[feature->feature_id()] = {body_id};
            break;
          }
          case cccad::geometry::v1::KernelFeature::kBoolean: {
            const auto& boolean = feature->boolean();
            const auto& target = find_body_state(bodies, boolean.target_body_id(), "boolean target");

            std::vector<TopoDS_Shape> tool_shapes;
            tool_shapes.reserve(static_cast<std::size_t>(boolean.tool_body_ids_size()));
            for (const auto& tool_body_id : boolean.tool_body_ids()) {
              if (tool_body_id == boolean.target_body_id()) {
                throw std::runtime_error("boolean tool body cannot be the target body: " + tool_body_id);
              }
              tool_shapes.push_back(find_body_state(bodies, tool_body_id, "boolean tool").shape);
            }

            cccad::geometry::v1::BuildBooleanRequest build_request;
            build_request.mutable_context()->CopyFrom(request->context());
            build_request.set_feature_id(feature->feature_id());
            build_request.set_operation(boolean.operation());
            build_request.set_target_body_id(boolean.target_body_id());
            for (const auto& tool_body_id : boolean.tool_body_ids()) {
              build_request.add_tool_body_ids(tool_body_id);
            }
            build_request.mutable_output()->CopyFrom(request->output());

            bodies[boolean.target_body_id()] = BodyState{
                .shape = build_boolean_shape(build_request, target.shape, tool_shapes),
                .created_by_feature_id = feature->feature_id(),
            };
            for (const auto& tool_body_id : boolean.tool_body_ids()) {
              bodies.erase(tool_body_id);
            }
            body_ids_by_feature_id[feature->feature_id()] = {boolean.target_body_id()};
            break;
          }
          case cccad::geometry::v1::KernelFeature::kFillet: {
            const auto& fillet = feature->fillet();
            const auto& target = find_body_state(bodies, fillet.target_body_id(), "fillet target");

            cccad::geometry::v1::BuildFilletRequest build_request;
            build_request.mutable_context()->CopyFrom(request->context());
            build_request.set_feature_id(feature->feature_id());
            build_request.set_target_body_id(fillet.target_body_id());
            for (const auto& edge_ref : fillet.edge_refs()) {
              build_request.add_edge_refs(edge_ref);
            }
            build_request.set_radius(fillet.radius());
            build_request.mutable_output()->CopyFrom(request->output());

            bodies[fillet.target_body_id()] = BodyState{
                .shape = build_fillet_shape(build_request, target.shape),
                .created_by_feature_id = feature->feature_id(),
            };
            body_ids_by_feature_id[feature->feature_id()] = {fillet.target_body_id()};
            break;
          }
          case cccad::geometry::v1::KernelFeature::kChamfer: {
            const auto& chamfer = feature->chamfer();
            const auto& target = find_body_state(bodies, chamfer.target_body_id(), "chamfer target");

            cccad::geometry::v1::BuildChamferRequest build_request;
            build_request.mutable_context()->CopyFrom(request->context());
            build_request.set_feature_id(feature->feature_id());
            build_request.set_target_body_id(chamfer.target_body_id());
            for (const auto& edge_ref : chamfer.edge_refs()) {
              build_request.add_edge_refs(edge_ref);
            }
            build_request.set_distance(chamfer.distance());
            build_request.mutable_output()->CopyFrom(request->output());

            bodies[chamfer.target_body_id()] = BodyState{
                .shape = build_chamfer_shape(build_request, target.shape),
                .created_by_feature_id = feature->feature_id(),
            };
            body_ids_by_feature_id[feature->feature_id()] = {chamfer.target_body_id()};
            break;
          }
          case cccad::geometry::v1::KernelFeature::kPattern: {
            const auto& pattern = feature->pattern();
            const auto source_shapes = collect_pattern_source_shapes(pattern, bodies, body_ids_by_feature_id);

            cccad::geometry::v1::BuildPatternRequest build_request;
            build_request.mutable_context()->CopyFrom(request->context());
            build_request.set_feature_id(feature->feature_id());
            for (const auto& source_feature_id : pattern.source_feature_ids()) {
              build_request.add_source_feature_ids(source_feature_id);
            }
            for (const auto& source_body_id : pattern.source_body_ids()) {
              build_request.add_source_body_ids(source_body_id);
            }
            if (pattern.has_linear()) {
              build_request.mutable_linear()->CopyFrom(pattern.linear());
            } else if (pattern.has_circular()) {
              build_request.mutable_circular()->CopyFrom(pattern.circular());
            }
            build_request.mutable_output()->CopyFrom(request->output());

            const std::string body_id = feature->feature_id().empty() ? "body-pattern" : ("body-" + feature->feature_id());
            bodies[body_id] = BodyState{
                .shape = build_pattern_shape(build_request, source_shapes),
                .created_by_feature_id = feature->feature_id(),
            };
            body_ids_by_feature_id[feature->feature_id()] = {body_id};
            break;
          }
          case cccad::geometry::v1::KernelFeature::FEATURE_NOT_SET:
          default:
            throw std::runtime_error("feature must be extrude, hole, boolean, fillet, chamfer, or pattern");
        }
      } catch (const std::exception& ex) {
        add_diagnostic(response, *feature, "REBUILD_FEATURE_FAILED", "error", ex.what());
        response->set_success(false);
        return grpc::Status::OK;
      }
    }

    for (const auto& [body_id, body_state] : bodies) {
      const auto artifacts = artifact_writer_.write_requested_artifacts(*request, body_id, body_state.shape);

      auto* body = response->add_bodies();
      body->set_body_id(body_id);
      body->set_created_by_feature_id(body_state.created_by_feature_id);

      for (const auto& a : artifacts) {
        auto* top_artifact = response->add_artifacts();
        top_artifact->set_kind(a.kind);
        top_artifact->set_storage_key(a.storage_key);
        top_artifact->set_content_type(a.content_type);
        top_artifact->set_size_bytes(static_cast<std::int64_t>(a.size_bytes));

        auto* body_artifact = body->add_artifacts();
        body_artifact->CopyFrom(*top_artifact);
      }

      if (request->output().return_topology()) {
        add_shape_topology(body_id, body_state.shape, response->mutable_topology());
      }
    }

    response->set_success(true);
    return grpc::Status::OK;
  } catch (const std::exception& ex) {
    auto* d = response->add_diagnostics();
    d->set_code("REBUILD_FAILED");
    d->set_severity("error");
    d->set_message(ex.what());
    response->set_success(false);
    return grpc::Status::OK;
  }
}

grpc::Status GeometryKernelServiceImpl::GetTopology(
    grpc::ServerContext*,
    const cccad::geometry::v1::GetTopologyRequest* request,
    cccad::geometry::v1::GetTopologyResponse* response) {
  try {
    if (request->body_ids_size() == 0) {
      for (const auto& body : request->existing_bodies()) {
        const TopoDS_Shape shape = artifact_writer_.read_brep_artifact(body.brep());
        add_shape_topology(body.body_id(), shape, response->mutable_topology());
      }
    } else {
      for (const auto& body_id : request->body_ids()) {
        const auto& body = find_body_by_id(request->existing_bodies(), body_id, "topology");
        const TopoDS_Shape shape = artifact_writer_.read_brep_artifact(body.brep());
        add_shape_topology(body.body_id(), shape, response->mutable_topology());
      }
    }

    response->set_success(true);
    return grpc::Status::OK;
  } catch (const std::exception& ex) {
    auto* d = response->add_diagnostics();
    d->set_code("TOPOLOGY_FAILED");
    d->set_severity("error");
    d->set_message(ex.what());
    response->set_success(false);
    return grpc::Status::OK;
  }
}

grpc::Status GeometryKernelServiceImpl::GetFacePlane(
    grpc::ServerContext*,
    const cccad::geometry::v1::GetFacePlaneRequest* request,
    cccad::geometry::v1::GetFacePlaneResponse* response) {
  try {
    const std::string body_id = request->body_id().empty() ? request->body().body_id() : request->body_id();
    if (body_id.empty()) {
      throw std::runtime_error("body_id is required");
    }

    if (!request->body().body_id().empty() && request->body().body_id() != body_id) {
      throw std::runtime_error("request body_id does not match body.body_id");
    }

    const TopoDS_Shape shape = artifact_writer_.read_brep_artifact(request->body().brep());
    response->set_surface_type(get_face_plane(shape, request->face_id(), response->mutable_plane()));
    response->set_success(true);
    return grpc::Status::OK;
  } catch (const std::exception& ex) {
    auto* d = response->add_diagnostics();
    d->set_code("FACE_PLANE_FAILED");
    d->set_severity("error");
    d->set_message(ex.what());
    d->set_body_id(request->body_id());
    d->set_face_id(request->face_id());
    response->set_success(false);
    return grpc::Status::OK;
  }
}

} // namespace cccad::geometry
