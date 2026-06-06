#pragma once

#include <TopoDS_Shape.hxx>
#include "geometry_kernel.pb.h"

namespace cccad::geometry {

TopoDS_Shape build_extrude_shape(const cccad::geometry::v1::BuildExtrudeRequest& request);

} // namespace cccad::geometry
