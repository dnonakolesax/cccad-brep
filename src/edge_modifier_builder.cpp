#include "cccad/geometry/edge_modifier_builder.hpp"

#include <cmath>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include <BRepFilletAPI_MakeChamfer.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>

namespace cccad::geometry {

namespace {

std::vector<TopoDS_Edge> collect_edges(const TopoDS_Shape& shape) {
  std::vector<TopoDS_Edge> edges;
  for (TopExp_Explorer exp(shape, TopAbs_EDGE); exp.More(); exp.Next()) {
    edges.push_back(TopoDS::Edge(exp.Current()));
  }
  return edges;
}

bool is_all_ref(const std::string& ref) {
  return ref == "*" || ref == "all" || ref == "ALL";
}

int parse_edge_ref(const std::string& ref) {
  std::string value = ref;
  constexpr const char* prefix = "edge-";
  if (value.rfind(prefix, 0) == 0) {
    value = value.substr(5);
  }

  if (value.empty()) {
    throw std::runtime_error("edge reference is empty");
  }

  std::size_t parsed = 0;
  const int index = std::stoi(value, &parsed);
  if (parsed != value.size() || index <= 0) {
    throw std::runtime_error("edge reference must be '*'/'all' or a 1-based edge index like 'edge-1'");
  }

  return index;
}

template <typename Request>
std::vector<TopoDS_Edge> select_edges(const Request& request, const TopoDS_Shape& shape) {
  const auto edges = collect_edges(shape);
  if (edges.empty()) {
    throw std::runtime_error("target body has no edges");
  }

  if (request.edge_refs_size() == 0) {
    return edges;
  }

  for (const auto& ref : request.edge_refs()) {
    if (is_all_ref(ref)) {
      return edges;
    }
  }

  std::set<int> selected_indices;
  for (const auto& ref : request.edge_refs()) {
    const int index = parse_edge_ref(ref);
    if (index > static_cast<int>(edges.size())) {
      throw std::runtime_error("edge reference is out of range: " + ref);
    }
    selected_indices.insert(index - 1);
  }

  std::vector<TopoDS_Edge> selected;
  for (const int index : selected_indices) {
    selected.push_back(edges[static_cast<std::size_t>(index)]);
  }

  return selected;
}

} // namespace

TopoDS_Shape build_fillet_shape(const cccad::geometry::v1::BuildFilletRequest& request,
                                const TopoDS_Shape& target_shape) {
  if (request.target_body_id().empty()) {
    throw std::runtime_error("fillet target_body_id is required");
  }

  if (request.radius() <= 0.0 || !std::isfinite(request.radius())) {
    throw std::runtime_error("fillet radius must be a positive finite value");
  }

  BRepFilletAPI_MakeFillet fillet_maker(target_shape);
  for (const auto& edge : select_edges(request, target_shape)) {
    fillet_maker.Add(request.radius(), edge);
  }

  fillet_maker.Build();
  if (!fillet_maker.IsDone()) {
    throw std::runtime_error("OpenCascade fillet builder failed");
  }

  return fillet_maker.Shape();
}

TopoDS_Shape build_chamfer_shape(const cccad::geometry::v1::BuildChamferRequest& request,
                                 const TopoDS_Shape& target_shape) {
  if (request.target_body_id().empty()) {
    throw std::runtime_error("chamfer target_body_id is required");
  }

  if (request.distance() <= 0.0 || !std::isfinite(request.distance())) {
    throw std::runtime_error("chamfer distance must be a positive finite value");
  }

  BRepFilletAPI_MakeChamfer chamfer_maker(target_shape);
  for (const auto& edge : select_edges(request, target_shape)) {
    chamfer_maker.Add(request.distance(), edge);
  }

  chamfer_maker.Build();
  if (!chamfer_maker.IsDone()) {
    throw std::runtime_error("OpenCascade chamfer builder failed");
  }

  return chamfer_maker.Shape();
}

} // namespace cccad::geometry
