#include "cccad/geometry/extrude_builder.hpp"
#include "cccad/geometry/types.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepLib.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Ax2.hxx>
#include <gp_Ax3.hxx>
#include <gp_Circ.hxx>
#include <gp_Pln.hxx>
#include <gp_Trsf.hxx>

namespace cccad::geometry {

namespace {

TopoDS_Wire make_wire_from_curves(
    const cccad::geometry::v1::SketchPlane& plane,
    const google::protobuf::RepeatedPtrField<cccad::geometry::v1::ProfileCurve>& curves,
    const std::string& loop_name) {
  if (curves.empty()) {
    throw std::runtime_error(loop_name + " must not be empty");
  }

  if (curves.size() == 1 && curves.Get(0).has_circle()) {
    const auto& circle = curves.Get(0).circle();
    if (circle.radius() <= 0.0) {
      throw std::runtime_error("circle radius must be positive");
    }

    const gp_Pnt center = point_on_plane(plane, circle.center().x(), circle.center().y());
    const gp_Dir normal = to_gp_dir(normalize(from_proto(plane.normal()), "normal"));
    const gp_Dir x_axis = to_gp_dir(normalize(from_proto(plane.x_axis()), "x_axis"));
    const gp_Ax2 ax2(center, normal, x_axis);
    const gp_Circ circ(ax2, circle.radius());
    BRepBuilderAPI_MakeEdge edge_maker(circ);
    if (!edge_maker.IsDone()) {
      throw std::runtime_error("failed to make circle edge");
    }
    BRepBuilderAPI_MakeWire wire_maker(edge_maker.Edge());
    if (!wire_maker.IsDone()) {
      throw std::runtime_error("failed to make circle wire");
    }
    return wire_maker.Wire();
  }

  BRepBuilderAPI_MakePolygon polygon;
  for (const auto& curve : curves) {
    if (!curve.has_line()) {
      if (curve.has_arc()) {
        throw std::runtime_error("arc segments are declared in contract but not implemented in MVP extrude");
      }
      if (curve.has_circle()) {
        throw std::runtime_error("circle can only be used as a single-curve profile in MVP extrude");
      }
      throw std::runtime_error("unsupported profile curve");
    }

    const auto& line = curve.line();
    polygon.Add(point_on_plane(plane, line.start().x(), line.start().y()));
  }

  // Close by last segment end. BRepBuilderAPI_MakePolygon::Close() also closes to first point,
  // but adding the end point gives clearer failure modes for invalid ordered loops.
  const auto& last_line = curves.Get(curves.size() - 1).line();
  polygon.Add(point_on_plane(plane, last_line.end().x(), last_line.end().y()));
  polygon.Close();

  if (!polygon.IsDone()) {
    throw std::runtime_error("failed to make polygon wire; profile may be open or degenerate");
  }

  return polygon.Wire();
}

double signed_area_2d(const google::protobuf::RepeatedPtrField<cccad::geometry::v1::ProfileCurve>& curves,
                      const std::string& loop_name) {
  if (curves.size() == 1 && curves.Get(0).has_circle()) {
    constexpr double pi = 3.141592653589793238462643383279502884;
    const double radius = curves.Get(0).circle().radius();
    return pi * radius * radius;
  }

  double area = 0.0;
  for (const auto& curve : curves) {
    if (!curve.has_line()) {
      throw std::runtime_error(loop_name + " contains an unsupported curve for orientation");
    }

    const auto& line = curve.line();
    area += line.start().x() * line.end().y() - line.end().x() * line.start().y();
  }

  area *= 0.5;
  constexpr double eps = 1.0e-12;
  if (!std::isfinite(area) || std::abs(area) < eps) {
    throw std::runtime_error(loop_name + " must enclose a non-zero area");
  }

  return area;
}

TopoDS_Wire make_oriented_wire(
    const cccad::geometry::v1::SketchPlane& plane,
    const google::protobuf::RepeatedPtrField<cccad::geometry::v1::ProfileCurve>& curves,
    const std::string& loop_name,
    bool want_counter_clockwise) {
  TopoDS_Wire wire = make_wire_from_curves(plane, curves, loop_name);
  const bool is_counter_clockwise = signed_area_2d(curves, loop_name) > 0.0;
  if (is_counter_clockwise != want_counter_clockwise) {
    wire.Reverse();
  }
  return wire;
}

TopoDS_Face make_face_from_profile(const cccad::geometry::v1::SketchPlane& plane,
                                   const cccad::geometry::v1::SketchProfile& profile) {
  const gp_Pnt origin = point_on_plane(plane, 0.0, 0.0);
  const gp_Dir normal = to_gp_dir(normalize(from_proto(plane.normal()), "normal"));
  const gp_Dir x_axis = to_gp_dir(normalize(from_proto(plane.x_axis()), "x_axis"));
  const gp_Pln sketch_surface(gp_Ax3(origin, normal, x_axis));

  TopoDS_Wire outer_wire = make_oriented_wire(plane, profile.outer_loop(), "profile.outer_loop", true);
  BRepBuilderAPI_MakeFace face_maker(sketch_surface, outer_wire, true);

  for (int i = 0; i < profile.inner_loops_size(); ++i) {
    TopoDS_Wire inner_wire = make_oriented_wire(
        plane,
        profile.inner_loops(i).curves(),
        "profile.inner_loops[" + std::to_string(i) + "].curves",
        false);
    face_maker.Add(inner_wire);
  }

  if (!face_maker.IsDone()) {
    throw std::runtime_error("failed to make face from profile loops");
  }
  return face_maker.Face();
}

TopoDS_Shape translate_shape(const TopoDS_Shape& shape, const gp_Vec& vec) {
  gp_Trsf trsf;
  trsf.SetTranslation(vec);
  BRepBuilderAPI_Transform transformer(shape, trsf, true);
  if (!transformer.IsDone()) {
    throw std::runtime_error("failed to transform profile face");
  }
  return transformer.Shape();
}

} // namespace

TopoDS_Shape build_extrude_shape(const cccad::geometry::v1::BuildExtrudeRequest& request) {
  validate_plane(request.sketch_plane());

  const auto& params = request.parameters();
  if (params.depth() <= 0.0 || !std::isfinite(params.depth())) {
    throw std::runtime_error("extrude depth must be a positive finite value");
  }

  if (params.operation() != cccad::geometry::v1::SOLID_OPERATION_UNSPECIFIED &&
      params.operation() != cccad::geometry::v1::SOLID_OPERATION_NEW_BODY) {
    throw std::runtime_error("only operation=SOLID_OPERATION_NEW_BODY is supported in MVP geometry service");
  }

  if (params.draft_angle_rad() != 0.0) {
    throw std::runtime_error("draft angle is declared in contract but not implemented in MVP extrude");
  }

  const Vec3d n = normalize(from_proto(request.sketch_plane().normal()), "normal");
  const double depth = params.depth();

  TopoDS_Shape base_face = make_face_from_profile(request.sketch_plane(), request.profile());

  gp_Vec prism_vec;

  switch (params.direction()) {
    case cccad::geometry::v1::EXTRUDE_DIRECTION_UNSPECIFIED:
    case cccad::geometry::v1::EXTRUDE_DIRECTION_FORWARD:
      prism_vec = to_gp_vec(Vec3d{n.x * depth, n.y * depth, n.z * depth});
      break;
    case cccad::geometry::v1::EXTRUDE_DIRECTION_BACKWARD:
      prism_vec = to_gp_vec(Vec3d{-n.x * depth, -n.y * depth, -n.z * depth});
      break;
    case cccad::geometry::v1::EXTRUDE_DIRECTION_SYMMETRIC: {
      const gp_Vec shift = to_gp_vec(Vec3d{-n.x * depth / 2.0, -n.y * depth / 2.0, -n.z * depth / 2.0});
      base_face = translate_shape(base_face, shift);
      prism_vec = to_gp_vec(Vec3d{n.x * depth, n.y * depth, n.z * depth});
      break;
    }
    case cccad::geometry::v1::EXTRUDE_DIRECTION_THROUGH_ALL:
      throw std::runtime_error("through-all extrude is declared in contract but not implemented in MVP extrude");
    default:
      throw std::runtime_error("unsupported extrude direction enum value");
  }

  BRepPrimAPI_MakePrism prism_maker(base_face, prism_vec, false, true);
  prism_maker.Build();
  if (!prism_maker.IsDone()) {
    throw std::runtime_error("OpenCascade prism builder failed");
  }

  TopoDS_Shape result = prism_maker.Shape();
  BRepLib::BuildCurves3d(result);
  return result;
}

} // namespace cccad::geometry
