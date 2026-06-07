#pragma once

#include <string>

#include <TopoDS_Shape.hxx>
#include "geometry_kernel.pb.h"

namespace cccad::geometry {

void add_shape_topology(const std::string& body_id,
                        const TopoDS_Shape& shape,
                        cccad::geometry::v1::TopologySummary* topology);

std::string get_face_plane(const TopoDS_Shape& shape,
                           const std::string& face_id,
                           cccad::geometry::v1::SketchPlane* plane);

} // namespace cccad::geometry
