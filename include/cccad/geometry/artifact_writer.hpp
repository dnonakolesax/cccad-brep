#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include <TopoDS_Shape.hxx>
#include "geometry_kernel.pb.h"

namespace cccad::geometry {

struct ArtifactWriteResult {
  std::string kind;
  std::string storage_key;
  std::string content_type;
  std::uintmax_t size_bytes{};
};

class ArtifactWriter {
public:
  explicit ArtifactWriter(std::filesystem::path storage_root);

  std::vector<ArtifactWriteResult> write_requested_artifacts(
      const cccad::geometry::v1::BuildExtrudeRequest& request,
      const TopoDS_Shape& shape) const;

  std::vector<ArtifactWriteResult> write_requested_artifacts(
      const cccad::geometry::v1::BuildHoleRequest& request,
      const TopoDS_Shape& shape) const;

  std::vector<ArtifactWriteResult> write_requested_artifacts(
      const cccad::geometry::v1::BuildFilletRequest& request,
      const TopoDS_Shape& shape) const;

  std::vector<ArtifactWriteResult> write_requested_artifacts(
      const cccad::geometry::v1::BuildChamferRequest& request,
      const TopoDS_Shape& shape) const;

  std::vector<ArtifactWriteResult> write_requested_artifacts(
      const cccad::geometry::v1::BuildBooleanRequest& request,
      const TopoDS_Shape& shape) const;

  std::vector<ArtifactWriteResult> write_requested_artifacts(
      const cccad::geometry::v1::BuildPatternRequest& request,
      const TopoDS_Shape& shape) const;

  std::vector<ArtifactWriteResult> write_requested_artifacts(
      const cccad::geometry::v1::RebuildPartRequest& request,
      const std::string& body_id,
      const TopoDS_Shape& shape) const;

  TopoDS_Shape read_brep_artifact(const cccad::geometry::v1::ArtifactRef& artifact) const;

private:
  std::filesystem::path storage_root_;

  ArtifactWriteResult write_brep(const std::string& storage_key, const TopoDS_Shape& shape) const;
  ArtifactWriteResult write_mesh_json(const std::string& storage_key, const TopoDS_Shape& shape,
                                      double linear_deflection,
                                      double angular_deflection) const;
};

} // namespace cccad::geometry
