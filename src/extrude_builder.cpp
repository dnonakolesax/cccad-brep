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
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Ax2.hxx>
#include <gp_Circ.hxx>
#include <gp_Trsf.hxx>

namespace cccad::geometry {

namespace {

TopoDS_Wire make_wire_from_profile(const cccad::geometry::v1::SketchPlane& plane,
                                   const cccad::geometry::v1::SketchProfile& profile) {
  if (profile.outer_loop_size() == 0) {
    throw std::runtime_error("profile.outer_loop must not be empty");
  }

  if (profile.inner_loops_size() > 0) {
    throw std::runtime_error("inner loops are not supported in MVP extrude");
  }

  if (profile.outer_loop_size() == 1 && profile.outer_loop(0).has_circle()) {
    const auto& circle = profile.outer_loop(0).circle();
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
  for (const auto& curve : profile.outer_loop()) {
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
  const auto& last_line = profile.outer_loop(profile.outer_loop_size() - 1).line();
  polygon.Add(point_on_plane(plane, last_line.end().x(), last_line.end().y()));
  polygon.Close();

  if (!polygon.IsDone()) {
    throw std::runtime_error("failed to make polygon wire; profile may be open or degenerate");
  }

  return polygon.Wire();
}

TopoDS_Face make_face_from_profile(const cccad::geometry::v1::SketchPlane& plane,
                                   const cccad::geometry::v1::SketchProfile& profile) {
  TopoDS_Wire wire = make_wire_from_profile(plane, profile);
  BRepBuilderAPI_MakeFace face_maker(wire);
  if (!face_maker.IsDone()) {
    throw std::runtime_error("failed to make face from profile wire");
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

  return prism_maker.Shape();
}

} // namespace cccad::geometry
