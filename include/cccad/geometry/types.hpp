#pragma once

#include <cmath>
#include <stdexcept>
#include <string>

#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <gp_Dir.hxx>

#include "geometry_kernel.pb.h"

namespace cccad::geometry {

struct Vec3d {
  double x{};
  double y{};
  double z{};
};

inline Vec3d from_proto(const cccad::geometry::v1::Vec3& v) {
  return Vec3d{v.x(), v.y(), v.z()};
}

inline double dot(const Vec3d& a, const Vec3d& b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline double length(const Vec3d& v) {
  return std::sqrt(dot(v, v));
}

inline Vec3d normalize(const Vec3d& v, const char* name) {
  constexpr double eps = 1e-12;
  const double l = length(v);
  if (!std::isfinite(l) || l < eps) {
    throw std::runtime_error(std::string(name) + " must be a non-zero finite vector");
  }
  return Vec3d{v.x / l, v.y / l, v.z / l};
}

inline Vec3d cross(const Vec3d& a, const Vec3d& b) {
  return Vec3d{
    a.y * b.z - a.z * b.y,
    a.z * b.x - a.x * b.z,
    a.x * b.y - a.y * b.x,
  };
}

inline gp_Vec to_gp_vec(const Vec3d& v) {
  return gp_Vec(v.x, v.y, v.z);
}

inline gp_Dir to_gp_dir(const Vec3d& v) {
  return gp_Dir(v.x, v.y, v.z);
}

inline gp_Pnt point_on_plane(const cccad::geometry::v1::SketchPlane& plane, double x, double y) {
  const auto o = from_proto(plane.origin());
  const auto xa = from_proto(plane.x_axis());
  const auto ya = from_proto(plane.y_axis());
  return gp_Pnt(
    o.x + xa.x * x + ya.x * y,
    o.y + xa.y * x + ya.y * y,
    o.z + xa.z * x + ya.z * y
  );
}

inline void validate_plane(const cccad::geometry::v1::SketchPlane& plane) {
  constexpr double eps = 1e-6;
  const auto x = normalize(from_proto(plane.x_axis()), "x_axis");
  const auto y = normalize(from_proto(plane.y_axis()), "y_axis");
  const auto n = normalize(from_proto(plane.normal()), "normal");

  if (std::abs(dot(x, y)) > eps) {
    throw std::runtime_error("x_axis and y_axis must be perpendicular");
  }
  if (std::abs(dot(x, n)) > eps) {
    throw std::runtime_error("x_axis and normal must be perpendicular");
  }
  if (std::abs(dot(y, n)) > eps) {
    throw std::runtime_error("y_axis and normal must be perpendicular");
  }

  const auto c = normalize(cross(x, y), "cross(x_axis, y_axis)");
  if (std::abs(c.x - n.x) > eps || std::abs(c.y - n.y) > eps || std::abs(c.z - n.z) > eps) {
    throw std::runtime_error("normal must match cross(x_axis, y_axis)");
  }
}

} // namespace cccad::geometry
