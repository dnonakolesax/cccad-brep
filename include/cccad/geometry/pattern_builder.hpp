#pragma once

#include <vector>

#include <TopoDS_Shape.hxx>
#include "geometry_kernel.pb.h"

namespace cccad::geometry {

TopoDS_Shape build_pattern_shape(const cccad::geometry::v1::BuildPatternRequest& request,
                                 const std::vector<TopoDS_Shape>& source_shapes);

} // namespace cccad::geometry
