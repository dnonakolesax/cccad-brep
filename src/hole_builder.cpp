#include "cccad/geometry/hole_builder.hpp"
#include "cccad/geometry/types.hpp"

#include <cmath>
#include <stdexcept>

#include <BRepAlgoAPI_Cut.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepLib.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <Precision.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Ax2.hxx>
#include <gp_Circ.hxx>
#include <gp_Trsf.hxx>

namespace cccad::geometry {

namespace {

TopoDS_Face make_hole_face(const cccad::geometry::v1::SketchPlane& plane,
                           const cccad::geometry::v1::Vec2& center,
                           double radius) {
  const gp_Pnt p = point_on_plane(plane, center.x(), center.y());
  const gp_Dir normal = to_gp_dir(normalize(from_proto(plane.normal()), "normal"));
  const gp_Dir x_axis = to_gp_dir(normalize(from_proto(plane.x_axis()), "x_axis"));
  const gp_Circ circle(gp_Ax2(p, normal, x_axis), radius);

  BRepBuilderAPI_MakeEdge edge_maker(circle);
  if (!edge_maker.IsDone()) {
    throw std::runtime_error("failed to make hole circle edge");
  }

  BRepBuilderAPI_MakeWire wire_maker(edge_maker.Edge());
  if (!wire_maker.IsDone()) {
    throw std::runtime_error("failed to make hole circle wire");
  }

  BRepBuilderAPI_MakeFace face_maker(wire_maker.Wire());
  if (!face_maker.IsDone()) {
    throw std::runtime_error("failed to make hole cutting face");
  }

  return face_maker.Face();
}

TopoDS_Shape translate_shape(const TopoDS_Shape& shape, const gp_Vec& vec) {
  gp_Trsf trsf;
  trsf.SetTranslation(vec);
  BRepBuilderAPI_Transform transformer(shape, trsf, true);
  if (!transformer.IsDone()) {
    throw std::runtime_error("failed to transform hole cutting face");
  }
  return transformer.Shape();
}

} // namespace

TopoDS_Shape build_hole_shape(const cccad::geometry::v1::BuildHoleRequest& request,
                              const TopoDS_Shape& target_shape) {
  validate_plane(request.sketch_plane());

  const auto& params = request.parameters();
  if (params.diameter() <= 0.0 || !std::isfinite(params.diameter())) {
    throw std::runtime_error("hole diameter must be a positive finite value");
  }

  if (params.target_body_id().empty()) {
    throw std::runtime_error("hole target_body_id is required");
  }

  constexpr double through_all_depth = 1.0e6;
  const double depth = params.through_all() ? through_all_depth : params.depth();
  if (depth <= 0.0 || !std::isfinite(depth)) {
    throw std::runtime_error("hole depth must be a positive finite value unless through_all is true");
  }

  const Vec3d n = normalize(from_proto(request.sketch_plane().normal()), "normal");
  TopoDS_Shape cutter_face = make_hole_face(request.sketch_plane(), request.center(), params.diameter() / 2.0);

  gp_Vec prism_vec;
  const auto direction = params.direction();
  if (params.through_all()) {
    const gp_Vec shift = to_gp_vec(Vec3d{-n.x * depth / 2.0, -n.y * depth / 2.0, -n.z * depth / 2.0});
    cutter_face = translate_shape(cutter_face, shift);
    prism_vec = to_gp_vec(Vec3d{n.x * depth, n.y * depth, n.z * depth});
  } else {
    switch (direction) {
      case cccad::geometry::v1::EXTRUDE_DIRECTION_UNSPECIFIED:
      case cccad::geometry::v1::EXTRUDE_DIRECTION_FORWARD:
        prism_vec = to_gp_vec(Vec3d{n.x * depth, n.y * depth, n.z * depth});
        break;
      case cccad::geometry::v1::EXTRUDE_DIRECTION_BACKWARD:
        prism_vec = to_gp_vec(Vec3d{-n.x * depth, -n.y * depth, -n.z * depth});
        break;
      case cccad::geometry::v1::EXTRUDE_DIRECTION_SYMMETRIC: {
        const gp_Vec shift = to_gp_vec(Vec3d{-n.x * depth / 2.0, -n.y * depth / 2.0, -n.z * depth / 2.0});
        cutter_face = translate_shape(cutter_face, shift);
        prism_vec = to_gp_vec(Vec3d{n.x * depth, n.y * depth, n.z * depth});
        break;
      }
      case cccad::geometry::v1::EXTRUDE_DIRECTION_THROUGH_ALL:
        throw std::runtime_error("use hole through_all=true instead of EXTRUDE_DIRECTION_THROUGH_ALL");
      default:
        throw std::runtime_error("unsupported hole direction enum value");
    }
  }

  BRepPrimAPI_MakePrism prism_maker(cutter_face, prism_vec, false, true);
  prism_maker.Build();
  if (!prism_maker.IsDone()) {
    throw std::runtime_error("OpenCascade hole cutter prism builder failed");
  }

  BRepAlgoAPI_Cut cut_maker(target_shape, prism_maker.Shape());
  cut_maker.Build();
  if (!cut_maker.IsDone()) {
    throw std::runtime_error("OpenCascade hole cut failed");
  }

  cut_maker.SimplifyResult(true, true, Precision::Angular());
  TopoDS_Shape result = cut_maker.Shape();
  BRepLib::BuildCurves3d(result);
  return result;
}

} // namespace cccad::geometry
