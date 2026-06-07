#pragma once

#include <TopoDS_Shape.hxx>
#include "geometry_kernel.pb.h"

namespace cccad::geometry {

TopoDS_Shape build_hole_shape(const cccad::geometry::v1::BuildHoleRequest& request,
                              const TopoDS_Shape& target_shape);

} // namespace cccad::geometry
