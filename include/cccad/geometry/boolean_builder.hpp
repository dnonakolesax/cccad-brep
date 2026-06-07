#pragma once

#include <vector>

#include <TopoDS_Shape.hxx>
#include "geometry_kernel.pb.h"

namespace cccad::geometry {

TopoDS_Shape build_boolean_shape(const cccad::geometry::v1::BuildBooleanRequest& request,
                                 const TopoDS_Shape& target_shape,
                                 const std::vector<TopoDS_Shape>& tool_shapes);

} // namespace cccad::geometry
