#include "cccad/geometry/extrude_builder.hpp"
#include "cccad/geometry/types.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepLib.hxx>
#include <TopoDS_Edge.hxx>
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

const char* extrude_direction_name(cccad::geometry::v1::ExtrudeDirection direction) {
  switch (direction) {
    case cccad::geometry::v1::EXTRUDE_DIRECTION_UNSPECIFIED:
      return "EXTRUDE_DIRECTION_UNSPECIFIED";
    case cccad::geometry::v1::EXTRUDE_DIRECTION_FORWARD:
      return "EXTRUDE_DIRECTION_FORWARD";
    case cccad::geometry::v1::EXTRUDE_DIRECTION_BACKWARD:
      return "EXTRUDE_DIRECTION_BACKWARD";
    case cccad::geometry::v1::EXTRUDE_DIRECTION_SYMMETRIC:
      return "EXTRUDE_DIRECTION_SYMMETRIC";
    case cccad::geometry::v1::EXTRUDE_DIRECTION_THROUGH_ALL:
      return "EXTRUDE_DIRECTION_THROUGH_ALL";
    default:
      return "EXTRUDE_DIRECTION_UNKNOWN";
  }
}

void log_vec3d(const char* name, const Vec3d& v) {
  std::cerr << name << "=(" << v.x << ", " << v.y << ", " << v.z << ")";
}

struct Vec2d {
  double x{};
  double y{};
};

struct WireSegment {
  const cccad::geometry::v1::ProfileCurve* curve{};
  Vec2d start;
  Vec2d end;
};

double normalize_arc_end_angle(double start, double end, bool clockwise) {
  constexpr double two_pi = 2.0 * 3.141592653589793238462643383279502884;
  if (clockwise) {
    while (end >= start) {
      end -= two_pi;
    }
  } else {
    while (end <= start) {
      end += two_pi;
    }
  }

  const double sweep = end - start;
  if (std::abs(sweep) > 3.141592653589793238462643383279502884) {
    end -= std::copysign(two_pi, sweep);
  }
  return end;
}

Vec2d arc_point_2d(const cccad::geometry::v1::ArcSegment2D& arc, double angle) {
  return Vec2d{
      arc.center().x() + arc.radius() * std::cos(angle),
      arc.center().y() + arc.radius() * std::sin(angle),
  };
}

double distance_squared(Vec2d a, Vec2d b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return dx * dx + dy * dy;
}

bool almost_same_point(Vec2d a, Vec2d b) {
  constexpr double eps = 1.0e-5;
  return distance_squared(a, b) <= eps * eps;
}

bool point_lies_on_segment(Vec2d p, Vec2d a, Vec2d b) {
  constexpr double eps = 1.0e-5;
  const double abx = b.x - a.x;
  const double aby = b.y - a.y;
  const double apx = p.x - a.x;
  const double apy = p.y - a.y;
  const double cross = abx * apy - aby * apx;
  if (std::abs(cross) > eps) {
    return false;
  }

  const double dot = apx * abx + apy * aby;
  const double len_sq = abx * abx + aby * aby;
  return dot >= -eps && dot <= len_sq + eps;
}

Vec2d trimmed_line_endpoint(Vec2d endpoint, Vec2d other, const std::vector<Vec2d>& arc_endpoints) {
  constexpr double eps = 1.0e-5;
  Vec2d best = endpoint;
  double best_dist_sq = std::numeric_limits<double>::infinity();

  for (const Vec2d candidate : arc_endpoints) {
    const double dist_sq = distance_squared(endpoint, candidate);
    if (dist_sq <= eps * eps || dist_sq >= best_dist_sq || dist_sq >= distance_squared(other, candidate)) {
      continue;
    }
    if (point_lies_on_segment(candidate, endpoint, other)) {
      best = candidate;
      best_dist_sq = dist_sq;
    }
  }

  return best;
}

TopoDS_Edge make_line_edge(const cccad::geometry::v1::SketchPlane& plane,
                           Vec2d start,
                           Vec2d end) {
  BRepBuilderAPI_MakeEdge edge_maker(
      point_on_plane(plane, start.x, start.y),
      point_on_plane(plane, end.x, end.y));
  if (!edge_maker.IsDone()) {
    throw std::runtime_error("failed to make line edge");
  }
  return edge_maker.Edge();
}

TopoDS_Edge make_arc_edge(const cccad::geometry::v1::SketchPlane& plane,
                          const cccad::geometry::v1::ArcSegment2D& arc) {
  if (arc.radius() <= 0.0 || !std::isfinite(arc.radius())) {
    throw std::runtime_error("arc radius must be positive and finite");
  }
  if (!std::isfinite(arc.start_angle_rad()) || !std::isfinite(arc.end_angle_rad())) {
    throw std::runtime_error("arc angles must be finite");
  }

  const gp_Pnt center = point_on_plane(plane, arc.center().x(), arc.center().y());
  const gp_Dir normal = to_gp_dir(normalize(from_proto(plane.normal()), "normal"));
  const gp_Dir x_axis = to_gp_dir(normalize(from_proto(plane.x_axis()), "x_axis"));
  const gp_Ax2 ax2(center, normal, x_axis);
  const gp_Circ circ(ax2, arc.radius());
  const double end_angle = normalize_arc_end_angle(
      arc.start_angle_rad(), arc.end_angle_rad(), arc.clockwise());

  BRepBuilderAPI_MakeEdge edge_maker(circ, arc.start_angle_rad(), end_angle);
  if (!edge_maker.IsDone()) {
    throw std::runtime_error("failed to make arc edge");
  }
  return edge_maker.Edge();
}

std::vector<WireSegment> make_ordered_wire_segments(
    const google::protobuf::RepeatedPtrField<cccad::geometry::v1::ProfileCurve>& curves,
    const std::string& loop_name) {
  std::vector<Vec2d> arc_endpoints;
  for (const auto& curve : curves) {
    if (!curve.has_arc()) {
      continue;
    }
    const auto& arc = curve.arc();
    if (arc.radius() <= 0.0 || !std::isfinite(arc.radius())) {
      throw std::runtime_error("arc radius must be positive and finite");
    }
    if (!std::isfinite(arc.start_angle_rad()) || !std::isfinite(arc.end_angle_rad())) {
      throw std::runtime_error("arc angles must be finite");
    }
    const double end_angle = normalize_arc_end_angle(
        arc.start_angle_rad(), arc.end_angle_rad(), arc.clockwise());
    arc_endpoints.push_back(arc_point_2d(arc, arc.start_angle_rad()));
    arc_endpoints.push_back(arc_point_2d(arc, end_angle));
  }

  std::vector<WireSegment> segments;
  segments.reserve(curves.size());
  for (const auto& curve : curves) {
    if (curve.has_line()) {
      const auto& line = curve.line();
      const Vec2d original_start{line.start().x(), line.start().y()};
      const Vec2d original_end{line.end().x(), line.end().y()};
      const Vec2d start = trimmed_line_endpoint(original_start, original_end, arc_endpoints);
      const Vec2d end = trimmed_line_endpoint(original_end, original_start, arc_endpoints);
      segments.push_back(WireSegment{&curve, start, end});
    } else if (curve.has_arc()) {
      const auto& arc = curve.arc();
      const double end_angle = normalize_arc_end_angle(
          arc.start_angle_rad(), arc.end_angle_rad(), arc.clockwise());
      segments.push_back(WireSegment{
          &curve,
          arc_point_2d(arc, arc.start_angle_rad()),
          arc_point_2d(arc, end_angle),
      });
    } else if (curve.has_circle()) {
      throw std::runtime_error("circle can only be used as a single-curve profile in MVP extrude");
    } else {
      throw std::runtime_error("unsupported profile curve");
    }
  }

  if (segments.empty()) {
    throw std::runtime_error(loop_name + " must not be empty");
  }

  std::vector<WireSegment> ordered;
  std::vector<bool> used(segments.size(), false);
  ordered.reserve(segments.size());
  ordered.push_back(segments.front());
  used.front() = true;

  while (ordered.size() < segments.size()) {
    const Vec2d current = ordered.back().end;
    bool found = false;
    for (std::size_t i = 0; i < segments.size(); ++i) {
      if (used[i]) {
        continue;
      }
      WireSegment candidate = segments[i];
      if (almost_same_point(current, candidate.start)) {
        ordered.push_back(candidate);
        used[i] = true;
        found = true;
        break;
      }
      if (almost_same_point(current, candidate.end)) {
        std::swap(candidate.start, candidate.end);
        ordered.push_back(candidate);
        used[i] = true;
        found = true;
        break;
      }
    }
    if (!found) {
      throw std::runtime_error("failed to order profile curves into a closed wire");
    }
  }

  if (!almost_same_point(ordered.back().end, ordered.front().start)) {
    throw std::runtime_error("profile curves do not form a closed wire");
  }

  return ordered;
}

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

  BRepBuilderAPI_MakeWire wire_maker;
  const std::vector<WireSegment> segments = make_ordered_wire_segments(curves, loop_name);
  for (const WireSegment& segment : segments) {
    TopoDS_Edge edge;
    if (segment.curve->has_line()) {
      edge = make_line_edge(plane, segment.start, segment.end);
    } else if (segment.curve->has_arc()) {
      edge = make_arc_edge(plane, segment.curve->arc());
      const Vec2d original_start = arc_point_2d(segment.curve->arc(), segment.curve->arc().start_angle_rad());
      if (!almost_same_point(segment.start, original_start)) {
        edge.Reverse();
      }
    }
    wire_maker.Add(edge);
  }

  if (!wire_maker.IsDone()) {
    throw std::runtime_error("failed to make wire; profile may be open or degenerate");
  }

  TopoDS_Wire wire = wire_maker.Wire();
  BRepLib::BuildCurves3d(wire);
  return wire;
}

double signed_area_2d(const google::protobuf::RepeatedPtrField<cccad::geometry::v1::ProfileCurve>& curves,
                      const std::string& loop_name) {
  if (curves.size() == 1 && curves.Get(0).has_circle()) {
    constexpr double pi = 3.141592653589793238462643383279502884;
    const double radius = curves.Get(0).circle().radius();
    return pi * radius * radius;
  }

  double area = 0.0;
  const std::vector<WireSegment> segments = make_ordered_wire_segments(curves, loop_name);
  for (const WireSegment& segment : segments) {
    if (segment.curve->has_line()) {
      area += 0.5 * (segment.start.x * segment.end.y - segment.end.x * segment.start.y);
    } else if (segment.curve->has_arc()) {
      const auto& arc = segment.curve->arc();
      if (arc.radius() <= 0.0 || !std::isfinite(arc.radius())) {
        throw std::runtime_error("arc radius must be positive and finite");
      }
      if (!std::isfinite(arc.start_angle_rad()) || !std::isfinite(arc.end_angle_rad())) {
        throw std::runtime_error("arc angles must be finite");
      }

      const double original_start = arc.start_angle_rad();
      const double original_end = normalize_arc_end_angle(original_start, arc.end_angle_rad(), arc.clockwise());
      const bool is_reversed = !almost_same_point(segment.start, arc_point_2d(arc, original_start));
      const double start = is_reversed ? original_end : original_start;
      const double end = is_reversed ? original_start : original_end;
      const double radius = arc.radius();
      area += 0.5 * (
          radius * arc.center().x() * (std::sin(end) - std::sin(start)) -
          radius * arc.center().y() * (std::cos(end) - std::cos(start)) +
          radius * radius * (end - start));
    } else {
      throw std::runtime_error(loop_name + " contains an unsupported curve for orientation");
    }
  }

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

  std::cerr
      << "[extrude_builder] validated parameters:"
      << " feature_id=" << request.feature_id()
      << " depth=" << depth
      << " direction=" << extrude_direction_name(params.direction()) << "(" << params.direction() << ")"
      << " outer_curve_count=" << request.profile().outer_loop_size()
      << " inner_loop_count=" << request.profile().inner_loops_size()
      << '\n';
  std::cerr << "[extrude_builder] normalized sketch normal: ";
  log_vec3d("n", n);
  std::cerr << '\n';

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

  std::cerr
      << "[extrude_builder] prism_vec=("
      << prism_vec.X() << ", " << prism_vec.Y() << ", " << prism_vec.Z() << ")"
      << '\n';

  BRepPrimAPI_MakePrism prism_maker(base_face, prism_vec, false, true);
  prism_maker.Build();
  if (!prism_maker.IsDone()) {
    throw std::runtime_error("OpenCascade prism builder failed");
  }

  TopoDS_Shape result = prism_maker.Shape();
  BRepLib::BuildCurves3d(result);
  std::cerr << "[extrude_builder] prism build finished: feature_id=" << request.feature_id() << '\n';
  return result;
}

} // namespace cccad::geometry
