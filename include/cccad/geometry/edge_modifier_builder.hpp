#pragma once

#include <TopoDS_Shape.hxx>
#include "geometry_kernel.pb.h"

namespace cccad::geometry {

TopoDS_Shape build_fillet_shape(const cccad::geometry::v1::BuildFilletRequest& request,
                                const TopoDS_Shape& target_shape);

TopoDS_Shape build_chamfer_shape(const cccad::geometry::v1::BuildChamferRequest& request,
                                 const TopoDS_Shape& target_shape);

} // namespace cccad::geometry
