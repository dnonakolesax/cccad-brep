#include "cccad/geometry/topology_builder.hpp"

#include <map>
#include <stdexcept>
#include <string>

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRep_Tool.hxx>
#include <BRepTools_WireExplorer.hxx>
#include <GeomAbs_CurveType.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shell.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Dir.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>

namespace cccad::geometry {

namespace {

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

std::string vertex_id_for(const TopTools_IndexedMapOfShape& vertex_map, const TopoDS_Vertex& vertex) {
  if (vertex.IsNull()) {
    return "";
  }

  const int index = vertex_map.FindIndex(vertex);
  if (index <= 0) {
    return "";
  }

  return indexed_id("vertex", index);
}

void add_edge(const std::string& body_ref,
              const TopTools_IndexedMapOfShape& edge_map,
              const TopTools_IndexedMapOfShape& vertex_map,
              const TopoDS_Edge& edge,
              cccad::geometry::v1::Loop* loop) {
  const int edge_index = edge_map.FindIndex(edge);
  auto* out_edge = loop->add_edges();
  out_edge->set_edge_id(indexed_id("edge", edge_index));
  out_edge->set_stable_ref(body_ref + "/edge-" + std::to_string(edge_index));
  out_edge->set_curve_type(curve_type(edge));

  TopoDS_Vertex start;
  TopoDS_Vertex end;
  TopExp::Vertices(edge, start, end, true);
  out_edge->set_start_vertex_id(vertex_id_for(vertex_map, start));
  out_edge->set_end_vertex_id(vertex_id_for(vertex_map, end));
}

void add_face(const std::string& body_ref,
              const TopTools_IndexedMapOfShape& face_map,
              const TopTools_IndexedMapOfShape& edge_map,
              const TopTools_IndexedMapOfShape& vertex_map,
              const TopoDS_Face& face,
              cccad::geometry::v1::Shell* shell) {
  const int face_index = face_map.FindIndex(face);
  auto* out_face = shell->add_faces();
  out_face->set_face_id(indexed_id("face", face_index));
  out_face->set_stable_ref(body_ref + "/face-" + std::to_string(face_index));
  out_face->set_surface_type(surface_type(face));
  set_plane_if_planar(face, out_face);

  int loop_index = 0;
  for (TopExp_Explorer wire_exp(face, TopAbs_WIRE); wire_exp.More(); wire_exp.Next()) {
    ++loop_index;
    const TopoDS_Wire wire = TopoDS::Wire(wire_exp.Current());
    auto* loop = out_face->add_loops();
    loop->set_loop_id(indexed_id("loop", loop_index));
    loop->set_stable_ref(out_face->stable_ref() + "/loop-" + std::to_string(loop_index));

    bool added_edge = false;
    for (BRepTools_WireExplorer edge_exp(wire, face); edge_exp.More(); edge_exp.Next()) {
      add_edge(body_ref, edge_map, vertex_map, edge_exp.Current(), loop);
      added_edge = true;
    }

    if (!added_edge) {
      for (TopExp_Explorer edge_exp(wire, TopAbs_EDGE); edge_exp.More(); edge_exp.Next()) {
        add_edge(body_ref, edge_map, vertex_map, TopoDS::Edge(edge_exp.Current()), loop);
      }
    }
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

} // namespace

void add_shape_topology(const std::string& body_id,
                        const TopoDS_Shape& shape,
                        cccad::geometry::v1::TopologySummary* topology) {
  if (shape.IsNull()) {
    throw std::runtime_error("cannot extract topology from a null shape");
  }

  TopTools_IndexedMapOfShape face_map;
  TopTools_IndexedMapOfShape edge_map;
  TopTools_IndexedMapOfShape vertex_map;
  TopExp::MapShapes(shape, TopAbs_FACE, face_map);
  TopExp::MapShapes(shape, TopAbs_EDGE, edge_map);
  TopExp::MapShapes(shape, TopAbs_VERTEX, vertex_map);

  auto* body = topology->add_bodies();
  body->set_body_id(body_id);
  body->set_stable_ref(body_id);
  add_vertices(body_id, vertex_map, body);

  int shell_index = 0;
  for (TopExp_Explorer shell_exp(shape, TopAbs_SHELL); shell_exp.More(); shell_exp.Next()) {
    ++shell_index;
    const TopoDS_Shell shell = TopoDS::Shell(shell_exp.Current());
    auto* out_shell = body->add_shells();
    out_shell->set_shell_id(indexed_id("shell", shell_index));
    out_shell->set_stable_ref(body_id + "/shell-" + std::to_string(shell_index));

    for (TopExp_Explorer face_exp(shell, TopAbs_FACE); face_exp.More(); face_exp.Next()) {
      add_face(body_id, face_map, edge_map, vertex_map, TopoDS::Face(face_exp.Current()), out_shell);
    }
  }

  if (shell_index == 0 && face_map.Extent() > 0) {
    auto* out_shell = body->add_shells();
    out_shell->set_shell_id("shell-1");
    out_shell->set_stable_ref(body_id + "/shell-1");

    for (TopExp_Explorer face_exp(shape, TopAbs_FACE); face_exp.More(); face_exp.Next()) {
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

  TopTools_IndexedMapOfShape face_map;
  TopExp::MapShapes(shape, TopAbs_FACE, face_map);

  const int face_index = parse_face_id(face_id);
  if (face_index > face_map.Extent()) {
    throw std::runtime_error("face_id is out of range: " + face_id);
  }

  const TopoDS_Face face = TopoDS::Face(face_map(face_index));
  const std::string type = surface_type(face);
  BRepAdaptor_Surface surface(face, true);
  if (surface.GetType() == GeomAbs_Plane) {
    set_plane_from_occt(surface.Plane(), plane);
  }

  return type;
}

} // namespace cccad::geometry
