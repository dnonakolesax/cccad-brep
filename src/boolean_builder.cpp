#include "cccad/geometry/boolean_builder.hpp"

#include <stdexcept>

#include <BRepAlgoAPI_Common.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>

namespace cccad::geometry {

TopoDS_Shape build_boolean_shape(const cccad::geometry::v1::BuildBooleanRequest& request,
                                 const TopoDS_Shape& target_shape,
                                 const std::vector<TopoDS_Shape>& tool_shapes) {
  if (request.target_body_id().empty()) {
    throw std::runtime_error("boolean target_body_id is required");
  }

  if (tool_shapes.empty()) {
    throw std::runtime_error("boolean requires at least one tool body");
  }

  TopoDS_Shape result = target_shape;
  for (const auto& tool_shape : tool_shapes) {
    if (tool_shape.IsNull()) {
      throw std::runtime_error("boolean tool body shape is null");
    }

    switch (request.operation()) {
      case cccad::geometry::v1::BOOLEAN_OPERATION_UNITE: {
        BRepAlgoAPI_Fuse fuse_maker(result, tool_shape);
        fuse_maker.Build();
        if (!fuse_maker.IsDone()) {
          throw std::runtime_error("OpenCascade boolean unite failed");
        }
        result = fuse_maker.Shape();
        break;
      }
      case cccad::geometry::v1::BOOLEAN_OPERATION_SUBTRACT: {
        BRepAlgoAPI_Cut cut_maker(result, tool_shape);
        cut_maker.Build();
        if (!cut_maker.IsDone()) {
          throw std::runtime_error("OpenCascade boolean subtract failed");
        }
        result = cut_maker.Shape();
        break;
      }
      case cccad::geometry::v1::BOOLEAN_OPERATION_INTERSECT: {
        BRepAlgoAPI_Common common_maker(result, tool_shape);
        common_maker.Build();
        if (!common_maker.IsDone()) {
          throw std::runtime_error("OpenCascade boolean intersect failed");
        }
        result = common_maker.Shape();
        break;
      }
      case cccad::geometry::v1::BOOLEAN_OPERATION_UNSPECIFIED:
      default:
        throw std::runtime_error("boolean operation must be UNITE, SUBTRACT, or INTERSECT");
    }

    if (result.IsNull()) {
      throw std::runtime_error("OpenCascade boolean produced a null shape");
    }
  }

  return result;
}

} // namespace cccad::geometry
