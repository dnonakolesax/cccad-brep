#include "cccad/geometry/artifact_writer.hpp"

#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include <BRepTools.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRep_Tool.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopLoc_Location.hxx>
#include <Poly_Triangulation.hxx>
#include <Poly_Array1OfTriangle.hxx>
#include <gp_Pnt.hxx>

namespace cccad::geometry {

namespace {

std::string join_storage_key(const std::string& prefix, const std::string& filename) {
  if (prefix.empty()) {
    return filename;
  }
  if (prefix.back() == '/') {
    return prefix + filename;
  }
  return prefix + "/" + filename;
}

std::filesystem::path path_for_key(const std::filesystem::path& root, const std::string& key) {
  std::filesystem::path p = root;
  p /= std::filesystem::path(key).relative_path();
  return p;
}

void ensure_parent_dir(const std::filesystem::path& p) {
  const auto parent = p.parent_path();
  if (!parent.empty()) {
    std::filesystem::create_directories(parent);
  }
}

} // namespace

ArtifactWriter::ArtifactWriter(std::filesystem::path storage_root)
    : storage_root_(std::move(storage_root)) {}

std::vector<ArtifactWriteResult> ArtifactWriter::write_requested_artifacts(
    const cccad::geometry::v1::BuildExtrudeRequest& request,
    const TopoDS_Shape& shape) const {
  std::vector<ArtifactWriteResult> results;

  const auto& out = request.output();
  const std::string body_id = request.feature_id().empty() ? "body" : ("body-" + request.feature_id());
  const std::string prefix = out.storage_prefix();

  if (out.write_brep()) {
    results.push_back(write_brep(join_storage_key(prefix, body_id + ".brep"), shape));
  }

  if (out.write_mesh_json()) {
    double linear = out.mesh().linear_deflection() > 0.0 ? out.mesh().linear_deflection() : 0.1;
    double angular = out.mesh().angular_deflection_rad() > 0.0 ? out.mesh().angular_deflection_rad() : 0.0872664626;
    results.push_back(write_mesh_json(join_storage_key(prefix, body_id + ".mesh.json"), shape, linear, angular));
  }

  // GLB/STEP/STL are intentionally not implemented in the MVP core.
  // The contract already has flags for them, but the service will ignore them until exporters are added.
  return results;
}

ArtifactWriteResult ArtifactWriter::write_brep(const std::string& storage_key, const TopoDS_Shape& shape) const {
  const auto path = path_for_key(storage_root_, storage_key);
  ensure_parent_dir(path);

  if (!BRepTools::Write(shape, path.string().c_str())) {
    throw std::runtime_error("failed to write BREP artifact: " + path.string());
  }

  return ArtifactWriteResult{
      .kind = "brep",
      .storage_key = storage_key,
      .content_type = "application/x-brep",
      .size_bytes = std::filesystem::file_size(path),
  };
}

ArtifactWriteResult ArtifactWriter::write_mesh_json(const std::string& storage_key,
                                                    const TopoDS_Shape& shape,
                                                    double linear_deflection,
                                                    double angular_deflection) const {
  const auto path = path_for_key(storage_root_, storage_key);
  ensure_parent_dir(path);

  BRepMesh_IncrementalMesh mesher(shape, linear_deflection, false, angular_deflection, true);
  mesher.Perform();
  if (!mesher.IsDone()) {
    throw std::runtime_error("OpenCascade meshing failed");
  }

  std::ofstream out(path);
  if (!out) {
    throw std::runtime_error("failed to open mesh JSON artifact for writing: " + path.string());
  }

  out << std::setprecision(17);
  out << "{\n  \"vertices\": [";

  bool first_vertex = true;
  std::vector<std::array<int, 3>> triangles;
  int vertex_offset = 0;

  for (TopExp_Explorer exp(shape, TopAbs_FACE); exp.More(); exp.Next()) {
    const TopoDS_Face face = TopoDS::Face(exp.Current());
    TopLoc_Location loc;
    Handle(Poly_Triangulation) tri = BRep_Tool::Triangulation(face, loc);
    if (tri.IsNull()) {
      continue;
    }

    const gp_Trsf trsf = loc.Transformation();
    const int lower = tri->Nodes().Lower();
    const int upper = tri->Nodes().Upper();

    for (int i = lower; i <= upper; ++i) {
      gp_Pnt p = tri->Node(i).Transformed(trsf);
      if (!first_vertex) {
        out << ",";
      }
      first_vertex = false;
      out << "\n    [" << p.X() << ", " << p.Y() << ", " << p.Z() << "]";
    }

    const auto& tris = tri->Triangles();
    for (int i = tris.Lower(); i <= tris.Upper(); ++i) {
      int n1, n2, n3;
      tris.Value(i).Get(n1, n2, n3);
      triangles.push_back({vertex_offset + (n1 - lower), vertex_offset + (n2 - lower), vertex_offset + (n3 - lower)});
    }

    vertex_offset += (upper - lower + 1);
  }

  out << "\n  ],\n  \"triangles\": [";
  for (std::size_t i = 0; i < triangles.size(); ++i) {
    if (i > 0) {
      out << ",";
    }
    out << "\n    [" << triangles[i][0] << ", " << triangles[i][1] << ", " << triangles[i][2] << "]";
  }
  out << "\n  ]\n}\n";
  out.close();

  return ArtifactWriteResult{
      .kind = "mesh_json",
      .storage_key = storage_key,
      .content_type = "application/json",
      .size_bytes = std::filesystem::file_size(path),
  };
}

} // namespace cccad::geometry
