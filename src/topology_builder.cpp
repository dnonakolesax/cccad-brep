#include "cccad/geometry/topology_builder.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepLib.hxx>
#include <BRep_Tool.hxx>
#include <BRepTools.hxx>
#include <BRepTools_WireExplorer.hxx>
#include <GeomAbs_CurveType.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <TopAbs_Orientation.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Shell.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Circ.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Dir.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <gp_XYZ.hxx>

namespace cccad::geometry {

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;

std::string indexed_id(const std::string& prefix, int index) {
  return prefix + "-" + std::to_string(index);
}

void set_vec3(cccad::geometry::v1::Vec3* out, const gp_Pnt& point) {
  out->set_x(point.X());
  out->set_y(point.Y());
  out->set_z(point.Z());
}

void set_vec3(cccad::geometry::v1::Vec3* out, const gp_Dir& dir) {
  out->set_x(dir.X());
  out->set_y(dir.Y());
  out->set_z(dir.Z());
}

void set_vec3(cccad::geometry::v1::Vec3* out, const gp_XYZ& xyz) {
  out->set_x(xyz.X());
  out->set_y(xyz.Y());
  out->set_z(xyz.Z());
}

std::string surface_type(const TopoDS_Face& face) {
  BRepAdaptor_Surface surface(face, true);
  switch (surface.GetType()) {
    case GeomAbs_Plane:
      return "plane";
    case GeomAbs_Cylinder:
      return "cylinder";
    case GeomAbs_Cone:
      return "cone";
    case GeomAbs_Sphere:
      return "sphere";
    case GeomAbs_BSplineSurface:
      return "bspline";
    default:
      return "unknown";
  }
}

void set_plane_from_occt(const gp_Pln& plane, cccad::geometry::v1::SketchPlane* out_plane) {
  out_plane->set_kind("CUSTOM");
  set_vec3(out_plane->mutable_origin(), plane.Location());
  set_vec3(out_plane->mutable_x_axis(), plane.XAxis().Direction());
  set_vec3(out_plane->mutable_y_axis(), plane.YAxis().Direction());
  set_vec3(out_plane->mutable_normal(), plane.Axis().Direction());
}

int parse_face_id(const std::string& face_id) {
  std::string value = face_id;
  const std::size_t slash = value.rfind('/');
  if (slash != std::string::npos) {
    value = value.substr(slash + 1);
  }

  constexpr const char* prefix = "face-";
  if (value.rfind(prefix, 0) == 0) {
    value = value.substr(5);
  }

  if (value.empty()) {
    throw std::runtime_error("face_id is required");
  }

  std::size_t parsed = 0;
  const int index = std::stoi(value, &parsed);
  if (parsed != value.size() || index <= 0) {
    throw std::runtime_error("face_id must be a 1-based face index like 'face-1'");
  }

  return index;
}

void set_plane_if_planar(const TopoDS_Face& face, cccad::geometry::v1::Face* out) {
  BRepAdaptor_Surface surface(face, true);
  if (surface.GetType() != GeomAbs_Plane) {
    return;
  }

  const gp_Pln plane = surface.Plane();
  set_plane_from_occt(plane, out->mutable_plane());
}

void set_cylinder_if_cylindrical(const TopoDS_Face& face, cccad::geometry::v1::Face* out) {
  BRepAdaptor_Surface surface(face, true);
  if (surface.GetType() != GeomAbs_Cylinder) {
    return;
  }

  const gp_Cylinder cylinder = surface.Cylinder();
  auto* out_cylinder = out->mutable_cylinder();
  set_vec3(out_cylinder->mutable_origin(), cylinder.Axis().Location());
  set_vec3(out_cylinder->mutable_axis(), cylinder.Axis().Direction());
  out_cylinder->set_radius(cylinder.Radius());
}

std::string curve_type(const TopoDS_Edge& edge) {
  BRepAdaptor_Curve curve(edge);
  switch (curve.GetType()) {
    case GeomAbs_Line:
      return "line";
    case GeomAbs_Circle:
      return "circle";
    case GeomAbs_Ellipse:
      return "ellipse";
    case GeomAbs_BSplineCurve:
      return "bspline";
    default:
      return "unknown";
  }
}

std::string orientation_type(const TopoDS_Shape& shape) {
  switch (shape.Orientation()) {
    case TopAbs_FORWARD:
      return "forward";
    case TopAbs_REVERSED:
      return "reversed";
    case TopAbs_INTERNAL:
      return "internal";
    case TopAbs_EXTERNAL:
      return "external";
    default:
      return "unknown";
  }
}

int shape_index_ignore_orientation(const TopTools_IndexedMapOfShape& map, const TopoDS_Shape& shape) {
  int index = map.FindIndex(shape);
  if (index > 0) {
    return index;
  }

  TopoDS_Shape forward = shape;
  forward.Orientation(TopAbs_FORWARD);
  index = map.FindIndex(forward);
  if (index > 0) {
    return index;
  }

  TopoDS_Shape reversed = shape;
  reversed.Orientation(TopAbs_REVERSED);
  return map.FindIndex(reversed);
}

std::string vertex_id_for(const TopTools_IndexedMapOfShape& vertex_map, const TopoDS_Vertex& vertex) {
  if (vertex.IsNull()) {
    return "";
  }

  const int index = shape_index_ignore_orientation(vertex_map, vertex);
  if (index <= 0) {
    return "";
  }

  return indexed_id("vertex", index);
}

void set_circle_if_circular(const TopoDS_Edge& edge, cccad::geometry::v1::Edge* out) {
  BRepAdaptor_Curve curve(edge);
  if (curve.GetType() != GeomAbs_Circle) {
    return;
  }

  const gp_Circ circle = curve.Circle();
  auto* out_circle = out->mutable_circle();
  set_vec3(out_circle->mutable_center(), circle.Location());
  set_vec3(out_circle->mutable_normal(), circle.Axis().Direction());
  out_circle->set_radius(circle.Radius());
}

bool loop_edges_closed(const cccad::geometry::v1::Loop& loop) {
  if (loop.edges_size() == 0) {
    return false;
  }

  if (loop.edges_size() == 1 && loop.edges(0).start_vertex_id() == loop.edges(0).end_vertex_id()) {
    return true;
  }

  for (int i = 0; i < loop.edges_size(); ++i) {
    const auto& current = loop.edges(i);
    const auto& next = loop.edges((i + 1) % loop.edges_size());
    if (current.end_vertex_id().empty() || next.start_vertex_id().empty()) {
      return false;
    }
    if (current.end_vertex_id() != next.start_vertex_id()) {
      return false;
    }
  }

  return true;
}

std::pair<std::string, std::string> edge_vertex_ids(const TopTools_IndexedMapOfShape& vertex_map,
                                                    const TopoDS_Edge& edge) {
  TopoDS_Vertex start;
  TopoDS_Vertex end;
  TopExp::Vertices(edge, start, end, true);
  return {vertex_id_for(vertex_map, start), vertex_id_for(vertex_map, end)};
}

TopoDS_Edge reversed_edge(TopoDS_Edge edge) {
  edge.Reverse();
  return edge;
}

int add_edge(const std::string& body_ref,
             const TopTools_IndexedMapOfShape& edge_map,
             const TopTools_IndexedMapOfShape& vertex_map,
             const TopoDS_Edge& edge,
             std::map<std::string, int>& loop_edge_id_counts,
             cccad::geometry::v1::Loop* loop) {
  const int edge_index = shape_index_ignore_orientation(edge_map, edge);
  const std::string base_edge_id = edge_index > 0 ? indexed_id("edge", edge_index) : "edge-unmapped";

  int occurrence = ++loop_edge_id_counts[base_edge_id];
  std::string local_edge_id = base_edge_id;
  if (occurrence > 1) {
    local_edge_id += "#" + std::to_string(occurrence);
  }

  auto* out_edge = loop->add_edges();
  out_edge->set_edge_id(local_edge_id);
  out_edge->set_stable_ref(edge_index > 0 ? body_ref + "/edge-" + std::to_string(edge_index) : body_ref + "/edge-unmapped");
  out_edge->set_curve_type(curve_type(edge));
  out_edge->set_orientation(orientation_type(edge));

  const auto [start_vertex_id, end_vertex_id] = edge_vertex_ids(vertex_map, edge);
  out_edge->set_start_vertex_id(start_vertex_id);
  out_edge->set_end_vertex_id(end_vertex_id);
  set_circle_if_circular(edge, out_edge);

  return edge_index;
}

std::optional<gp_Pln> planar_surface(const TopoDS_Face& face) {
  BRepAdaptor_Surface surface(face, true);
  if (surface.GetType() != GeomAbs_Plane) {
    return std::nullopt;
  }
  return surface.Plane();
}

gp_Pnt vertex_point_by_id(const TopTools_IndexedMapOfShape& vertex_map, const std::string& vertex_id) {
  constexpr const char* prefix = "vertex-";
  if (vertex_id.rfind(prefix, 0) != 0) {
    return gp_Pnt(0.0, 0.0, 0.0);
  }

  const int index = std::stoi(vertex_id.substr(7));
  if (index <= 0 || index > vertex_map.Extent()) {
    return gp_Pnt(0.0, 0.0, 0.0);
  }

  return BRep_Tool::Pnt(TopoDS::Vertex(vertex_map(index)));
}

std::pair<double, double> project_to_plane_uv(const gp_Pln& plane, const gp_Pnt& point) {
  const gp_Vec delta(plane.Location(), point);
  const gp_Dir x_axis = plane.XAxis().Direction();
  const gp_Dir y_axis = plane.YAxis().Direction();
  return {delta.Dot(gp_Vec(x_axis)), delta.Dot(gp_Vec(y_axis))};
}

double loop_area_abs_on_plane(const cccad::geometry::v1::Loop& loop,
                              const TopTools_IndexedMapOfShape& vertex_map,
                              const gp_Pln& plane) {
  if (loop.edges_size() == 0) {
    return 0.0;
  }

  if (loop.edges_size() == 1 && loop.edges(0).has_circle()) {
    const double r = loop.edges(0).circle().radius();
    return kPi * r * r;
  }

  std::vector<std::pair<double, double>> points;
  points.reserve(static_cast<std::size_t>(loop.edges_size()));
  for (const auto& edge : loop.edges()) {
    if (edge.start_vertex_id().empty()) {
      continue;
    }
    points.push_back(project_to_plane_uv(plane, vertex_point_by_id(vertex_map, edge.start_vertex_id())));
  }

  if (points.size() < 3) {
    return 0.0;
  }

  double area = 0.0;
  for (std::size_t i = 0; i < points.size(); ++i) {
    const auto& a = points[i];
    const auto& b = points[(i + 1) % points.size()];
    area += a.first * b.second - b.first * a.second;
  }

  return std::abs(area * 0.5);
}

void classify_loop_roles(cccad::geometry::v1::Face* out_face,
                         const TopTools_IndexedMapOfShape& vertex_map,
                         const TopoDS_Face& face) {
  if (out_face->loops_size() == 0) {
    return;
  }

  int existing_outer = -1;
  for (int i = 0; i < out_face->loops_size(); ++i) {
    const auto& loop = out_face->loops(i);
    if (loop.role() == "outer" && loop.edges_size() > 0) {
      existing_outer = i;
      break;
    }
  }

  if (existing_outer < 0) {
    existing_outer = 0;
    if (const auto plane = planar_surface(face); plane.has_value()) {
      double best_area = -1.0;
      for (int i = 0; i < out_face->loops_size(); ++i) {
        const double area = loop_area_abs_on_plane(out_face->loops(i), vertex_map, *plane);
        if (area > best_area) {
          best_area = area;
          existing_outer = i;
        }
      }
    }
  }

  for (int i = 0; i < out_face->loops_size(); ++i) {
    auto* loop = out_face->mutable_loops(i);
    loop->set_role(i == existing_outer ? "outer" : "inner");
    loop->set_closed(loop_edges_closed(*loop));
  }
}

bool add_loop_from_wire(const std::string& body_ref,
                        const TopTools_IndexedMapOfShape& edge_map,
                        const TopTools_IndexedMapOfShape& vertex_map,
                        const TopoDS_Face& face,
                        const TopoDS_Wire& wire,
                        const std::string& role,
                        int loop_index,
                        cccad::geometry::v1::Face* out_face) {
  auto* loop = out_face->add_loops();
  loop->set_loop_id(indexed_id("loop", loop_index));
  loop->set_stable_ref(out_face->stable_ref() + "/loop-" + std::to_string(loop_index));
  loop->set_role(role);

  std::map<std::string, int> loop_edge_id_counts;
  for (BRepTools_WireExplorer edge_exp(wire, face); edge_exp.More(); edge_exp.Next()) {
    add_edge(body_ref, edge_map, vertex_map, edge_exp.Current(), loop_edge_id_counts, loop);
  }

  if (loop->edges_size() == 0) {
    for (TopExp_Explorer edge_exp(wire, TopAbs_EDGE); edge_exp.More(); edge_exp.Next()) {
      add_edge(body_ref, edge_map, vertex_map, TopoDS::Edge(edge_exp.Current()), loop_edge_id_counts, loop);
    }
  }

  if (loop->edges_size() == 0) {
    out_face->mutable_loops()->RemoveLast();
    return false;
  }

  loop->set_closed(loop_edges_closed(*loop));
  return true;
}

std::vector<TopoDS_Edge> collect_face_edges(const TopoDS_Face& face) {
  TopTools_IndexedMapOfShape unique_edges;
  TopExp::MapShapes(face, TopAbs_EDGE, unique_edges);

  std::vector<TopoDS_Edge> result;
  result.reserve(static_cast<std::size_t>(unique_edges.Extent()));
  for (int i = 1; i <= unique_edges.Extent(); ++i) {
    result.push_back(TopoDS::Edge(unique_edges(i)));
  }
  return result;
}

std::vector<std::vector<TopoDS_Edge>> build_edge_loops_by_connectivity(
    const std::vector<TopoDS_Edge>& input_edges,
    const TopTools_IndexedMapOfShape& vertex_map) {
  std::vector<TopoDS_Edge> unused = input_edges;
  std::vector<std::vector<TopoDS_Edge>> loops;

  while (!unused.empty()) {
    std::vector<TopoDS_Edge> loop;
    loop.push_back(unused.front());
    unused.erase(unused.begin());

    auto [start_id, current_end_id] = edge_vertex_ids(vertex_map, loop.front());

    // A full circle edge is a valid one-edge closed loop.
    if (!start_id.empty() && start_id == current_end_id) {
      loops.push_back(std::move(loop));
      continue;
    }

    while (!unused.empty() && !start_id.empty() && !current_end_id.empty() && current_end_id != start_id) {
      auto next_it = unused.end();
      bool reverse_next = false;

      for (auto it = unused.begin(); it != unused.end(); ++it) {
        const auto [candidate_start, candidate_end] = edge_vertex_ids(vertex_map, *it);
        if (candidate_start == current_end_id) {
          next_it = it;
          reverse_next = false;
          break;
        }
        if (candidate_end == current_end_id) {
          next_it = it;
          reverse_next = true;
          break;
        }
      }

      if (next_it == unused.end()) {
        break;
      }

      TopoDS_Edge next_edge = *next_it;
      unused.erase(next_it);
      if (reverse_next) {
        next_edge = reversed_edge(next_edge);
      }
      current_end_id = edge_vertex_ids(vertex_map, next_edge).second;
      loop.push_back(next_edge);
    }

    loops.push_back(std::move(loop));
  }

  return loops;
}

void add_loops_from_face_edges_fallback(const std::string& body_ref,
                                        const TopTools_IndexedMapOfShape& edge_map,
                                        const TopTools_IndexedMapOfShape& vertex_map,
                                        const TopoDS_Face& face,
                                        cccad::geometry::v1::Face* out_face) {
  const auto edge_loops = build_edge_loops_by_connectivity(collect_face_edges(face), vertex_map);

  int loop_index = out_face->loops_size();
  for (const auto& edge_loop : edge_loops) {
    if (edge_loop.empty()) {
      continue;
    }

    ++loop_index;
    auto* loop = out_face->add_loops();
    loop->set_loop_id(indexed_id("loop", loop_index));
    loop->set_stable_ref(out_face->stable_ref() + "/loop-" + std::to_string(loop_index));
    loop->set_role("unknown");

    std::map<std::string, int> loop_edge_id_counts;
    for (const auto& edge : edge_loop) {
      add_edge(body_ref, edge_map, vertex_map, edge, loop_edge_id_counts, loop);
    }

    if (loop->edges_size() == 0) {
      out_face->mutable_loops()->RemoveLast();
      --loop_index;
      continue;
    }

    loop->set_closed(loop_edges_closed(*loop));
  }
}

bool has_non_empty_outer_loop(const cccad::geometry::v1::Face& face) {
  for (const auto& loop : face.loops()) {
    if (loop.role() == "outer" && loop.edges_size() > 0) {
      return true;
    }
  }
  return false;
}


void remove_empty_loops(cccad::geometry::v1::Face* face) {
  for (int i = face->loops_size() - 1; i >= 0; --i) {
    if (face->loops(i).edges_size() == 0) {
      face->mutable_loops()->DeleteSubrange(i, 1);
    }
  }
}

struct PlanarVertexCandidate {
  std::string vertex_id;
  double u = 0.0;
  double v = 0.0;
};

double cross_2d(const PlanarVertexCandidate& o,
                const PlanarVertexCandidate& a,
                const PlanarVertexCandidate& b) {
  return (a.u - o.u) * (b.v - o.v) - (a.v - o.v) * (b.u - o.u);
}

std::vector<PlanarVertexCandidate> convex_hull(std::vector<PlanarVertexCandidate> points) {
  constexpr double eps = 1.0e-9;
  if (points.size() < 3) {
    return {};
  }

  std::sort(points.begin(), points.end(), [](const auto& a, const auto& b) {
    if (a.u == b.u) {
      return a.v < b.v;
    }
    return a.u < b.u;
  });

  std::vector<PlanarVertexCandidate> unique;
  unique.reserve(points.size());
  for (const auto& point : points) {
    if (!unique.empty() && std::abs(unique.back().u - point.u) < eps && std::abs(unique.back().v - point.v) < eps) {
      continue;
    }
    unique.push_back(point);
  }

  if (unique.size() < 3) {
    return {};
  }

  std::vector<PlanarVertexCandidate> lower;
  for (const auto& point : unique) {
    while (lower.size() >= 2 && cross_2d(lower[lower.size() - 2], lower.back(), point) <= eps) {
      lower.pop_back();
    }
    lower.push_back(point);
  }

  std::vector<PlanarVertexCandidate> upper;
  for (auto it = unique.rbegin(); it != unique.rend(); ++it) {
    while (upper.size() >= 2 && cross_2d(upper[upper.size() - 2], upper.back(), *it) <= eps) {
      upper.pop_back();
    }
    upper.push_back(*it);
  }

  lower.pop_back();
  upper.pop_back();
  lower.insert(lower.end(), upper.begin(), upper.end());
  return lower;
}

int next_loop_index(const cccad::geometry::v1::Face& face) {
  int result = face.loops_size() + 1;
  for (const auto& loop : face.loops()) {
    const std::string& loop_id = loop.loop_id();
    constexpr const char* prefix = "loop-";
    if (loop_id.rfind(prefix, 0) != 0) {
      continue;
    }
    try {
      result = std::max(result, std::stoi(loop_id.substr(5)) + 1);
    } catch (...) {
      // Keep generated fallback index.
    }
  }
  return result;
}

void add_synthetic_outer_loop_from_planar_hull(const std::string& body_ref,
                                               const TopTools_IndexedMapOfShape& vertex_map,
                                               const TopoDS_Face& face,
                                               cccad::geometry::v1::Face* out_face) {
  const auto plane = planar_surface(face);
  if (!plane.has_value()) {
    return;
  }

  constexpr double distance_tolerance = 1.0e-6;
  std::vector<PlanarVertexCandidate> candidates;
  candidates.reserve(static_cast<std::size_t>(vertex_map.Extent()));

  for (int i = 1; i <= vertex_map.Extent(); ++i) {
    const TopoDS_Vertex vertex = TopoDS::Vertex(vertex_map(i));
    const gp_Pnt point = BRep_Tool::Pnt(vertex);
    if (plane->Distance(point) > distance_tolerance) {
      continue;
    }

    const auto [u, v] = project_to_plane_uv(*plane, point);
    candidates.push_back(PlanarVertexCandidate{indexed_id("vertex", i), u, v});
  }

  const std::size_t candidate_count = candidates.size();
  std::vector<PlanarVertexCandidate> hull = convex_hull(std::move(candidates));
  if (hull.size() < 3) {
    std::cerr
        << "[topology_builder] unable to synthesize planar outer loop: face_id="
        << out_face->face_id()
        << " candidate_count=" << candidate_count
        << '\n';
    return;
  }

  const int loop_index = next_loop_index(*out_face);
  auto* loop = out_face->add_loops();
  loop->set_loop_id(indexed_id("loop", loop_index));
  loop->set_stable_ref(out_face->stable_ref() + "/loop-" + std::to_string(loop_index));
  loop->set_role("outer");
  loop->set_closed(true);

  for (std::size_t i = 0; i < hull.size(); ++i) {
    const auto& start = hull[i];
    const auto& end = hull[(i + 1) % hull.size()];
    auto* edge = loop->add_edges();
    const std::string edge_id = "synthetic-" + out_face->face_id() + "-loop-" + std::to_string(loop_index) + "-edge-" + std::to_string(i + 1);
    edge->set_edge_id(edge_id);
    edge->set_stable_ref(body_ref + "/" + edge_id);
    edge->set_curve_type("line");
    edge->set_start_vertex_id(start.vertex_id);
    edge->set_end_vertex_id(end.vertex_id);
    edge->set_orientation("forward");
  }

  std::cerr
      << "[topology_builder] synthesized planar outer loop: face_id="
      << out_face->face_id()
      << " vertex_count=" << hull.size()
      << '\n';
}

void add_face(const std::string& body_ref,
              const TopTools_IndexedMapOfShape& face_map,
              const TopTools_IndexedMapOfShape& edge_map,
              const TopTools_IndexedMapOfShape& vertex_map,
              const TopoDS_Face& face,
              cccad::geometry::v1::Shell* shell) {
  const int face_index = shape_index_ignore_orientation(face_map, face);
  auto* out_face = shell->add_faces();
  out_face->set_face_id(indexed_id("face", face_index));
  out_face->set_stable_ref(body_ref + "/face-" + std::to_string(face_index));
  out_face->set_surface_type(surface_type(face));
  set_plane_if_planar(face, out_face);
  set_cylinder_if_cylindrical(face, out_face);

  int loop_index = 0;
  const TopoDS_Wire outer_wire = BRepTools::OuterWire(face);
  for (TopExp_Explorer wire_exp(face, TopAbs_WIRE); wire_exp.More(); wire_exp.Next()) {
    const TopoDS_Wire wire = TopoDS::Wire(wire_exp.Current());
    const bool is_outer = !outer_wire.IsNull() && wire.IsSame(outer_wire);
    ++loop_index;
    add_loop_from_wire(
        body_ref,
        edge_map,
        vertex_map,
        face,
        wire,
        is_outer ? "outer" : "inner",
        loop_index,
        out_face);
  }

  // Some OCCT boolean/prism results can expose an empty wire placeholder when
  // 3D curves or p-curves are not yet rebuilt. Do not return that placeholder
  // to the frontend. Reconstruct loops from the face edges instead.
  if (out_face->loops_size() == 0 || !has_non_empty_outer_loop(*out_face)) {
    add_loops_from_face_edges_fallback(body_ref, edge_map, vertex_map, face, out_face);
  }

  remove_empty_loops(out_face);
  classify_loop_roles(out_face, vertex_map, face);
  remove_empty_loops(out_face);

  // Last-resort MVP recovery path. If OCCT exposes a planar support surface but
  // no usable outer wire, derive an outer boundary from the convex hull of body
  // vertices lying on the same support plane. This is intentionally conservative:
  // it only runs for planar faces with no non-empty outer loop and keeps existing
  // inner loops such as circular holes intact.
  if (!has_non_empty_outer_loop(*out_face)) {
    add_synthetic_outer_loop_from_planar_hull(body_ref, vertex_map, face, out_face);
    classify_loop_roles(out_face, vertex_map, face);
    remove_empty_loops(out_face);
  }
}

void add_vertices(const std::string& body_ref,
                  const TopTools_IndexedMapOfShape& vertex_map,
                  cccad::geometry::v1::Body* body) {
  for (int i = 1; i <= vertex_map.Extent(); ++i) {
    const TopoDS_Vertex vertex = TopoDS::Vertex(vertex_map(i));
    auto* out_vertex = body->add_vertices();
    out_vertex->set_vertex_id(indexed_id("vertex", i));
    out_vertex->set_stable_ref(body_ref + "/vertex-" + std::to_string(i));
    set_vec3(out_vertex->mutable_point(), BRep_Tool::Pnt(vertex));
  }
}

TopoDS_Shape normalize_shape_for_topology(const TopoDS_Shape& shape) {
  if (shape.IsNull()) {
    return shape;
  }

  // Rebuild missing 3D curves before exporting B-Rep topology. This is enough for
  // the topology DTO fallback below and avoids pulling TKShHealing into the MVP
  // container link stage.
  TopoDS_Shape result = shape;
  BRepLib::BuildCurves3d(result);
  return result;
}

} // namespace

void add_shape_topology(const std::string& body_id,
                        const TopoDS_Shape& shape,
                        cccad::geometry::v1::TopologySummary* topology) {
  if (shape.IsNull()) {
    throw std::runtime_error("cannot extract topology from a null shape");
  }

  const TopoDS_Shape topology_shape = normalize_shape_for_topology(shape);

  TopTools_IndexedMapOfShape face_map;
  TopTools_IndexedMapOfShape edge_map;
  TopTools_IndexedMapOfShape vertex_map;
  TopExp::MapShapes(topology_shape, TopAbs_FACE, face_map);
  TopExp::MapShapes(topology_shape, TopAbs_EDGE, edge_map);
  TopExp::MapShapes(topology_shape, TopAbs_VERTEX, vertex_map);

  auto* body = topology->add_bodies();
  body->set_body_id(body_id);
  body->set_stable_ref(body_id);
  add_vertices(body_id, vertex_map, body);

  int shell_index = 0;
  for (TopExp_Explorer shell_exp(topology_shape, TopAbs_SHELL); shell_exp.More(); shell_exp.Next()) {
    ++shell_index;
    const TopoDS_Shell shell = TopoDS::Shell(shell_exp.Current());
    auto* out_shell = body->add_shells();
    out_shell->set_shell_id(indexed_id("shell", shell_index));
    out_shell->set_stable_ref(body_id + "/shell-" + std::to_string(shell_index));

    for (TopExp_Explorer face_exp(shell, TopAbs_FACE); face_exp.More(); face_exp.Next()) {
      add_face(body_id, face_map, edge_map, vertex_map, TopoDS::Face(face_exp.Current()), out_shell);
    }
  }

  if (shell_index == 0) {
    auto* out_shell = body->add_shells();
    out_shell->set_shell_id("shell-1");
    out_shell->set_stable_ref(body_id + "/shell-1");
    for (TopExp_Explorer face_exp(topology_shape, TopAbs_FACE); face_exp.More(); face_exp.Next()) {
      add_face(body_id, face_map, edge_map, vertex_map, TopoDS::Face(face_exp.Current()), out_shell);
    }
  }
}

std::string get_face_plane(const TopoDS_Shape& shape,
                           const std::string& face_id,
                           cccad::geometry::v1::SketchPlane* plane) {
  if (shape.IsNull()) {
    throw std::runtime_error("cannot extract face plane from a null shape");
  }

  if (plane == nullptr) {
    throw std::runtime_error("output plane pointer is null");
  }

  const TopoDS_Shape topology_shape = normalize_shape_for_topology(shape);
  const int target_index = parse_face_id(face_id);

  TopTools_IndexedMapOfShape face_map;
  TopExp::MapShapes(topology_shape, TopAbs_FACE, face_map);
  if (target_index > face_map.Extent()) {
    throw std::runtime_error("face_id is outside topology face range: " + face_id);
  }

  const TopoDS_Face face = TopoDS::Face(face_map(target_index));
  BRepAdaptor_Surface surface(face, true);
  if (surface.GetType() != GeomAbs_Plane) {
    throw std::runtime_error("selected face is not planar");
  }

  set_plane_from_occt(surface.Plane(), plane);
  return "plane";
}

} // namespace cccad::geometry
