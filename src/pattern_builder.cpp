#include "cccad/geometry/pattern_builder.hpp"
#include "cccad/geometry/types.hpp"

#include <cmath>
#include <stdexcept>

#include <BRep_Builder.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <TopoDS_Compound.hxx>
#include <gp_Ax1.hxx>
#include <gp_Trsf.hxx>

namespace cccad::geometry {

namespace {

TopoDS_Shape transformed_shape(const TopoDS_Shape& shape, const gp_Trsf& trsf) {
  BRepBuilderAPI_Transform transformer(shape, trsf, true);
  if (!transformer.IsDone()) {
    throw std::runtime_error("failed to transform pattern source body");
  }
  return transformer.Shape();
}

void add_instance(BRep_Builder& builder,
                  TopoDS_Compound& compound,
                  const TopoDS_Shape& source_shape,
                  const gp_Trsf& trsf,
                  bool identity) {
  builder.Add(compound, identity ? source_shape : transformed_shape(source_shape, trsf));
}

void validate_count(int count) {
  if (count <= 0) {
    throw std::runtime_error("pattern count must be a positive integer");
  }
}

} // namespace

TopoDS_Shape build_pattern_shape(const cccad::geometry::v1::BuildPatternRequest& request,
                                 const std::vector<TopoDS_Shape>& source_shapes) {
  if (source_shapes.empty()) {
    throw std::runtime_error("pattern requires at least one source body");
  }

  for (const auto& source_shape : source_shapes) {
    if (source_shape.IsNull()) {
      throw std::runtime_error("pattern source body shape is null");
    }
  }

  BRep_Builder builder;
  TopoDS_Compound compound;
  builder.MakeCompound(compound);

  switch (request.parameters_case()) {
    case cccad::geometry::v1::BuildPatternRequest::kLinear: {
      const auto& params = request.linear();
      validate_count(params.count());

      if (!std::isfinite(params.spacing())) {
        throw std::runtime_error("linear pattern spacing must be finite");
      }

      const Vec3d direction = normalize(from_proto(params.direction()), "linear pattern direction");
      for (int instance_index = 0; instance_index < params.count(); ++instance_index) {
        gp_Trsf trsf;
        if (instance_index != 0) {
          trsf.SetTranslation(to_gp_vec(Vec3d{
              direction.x * params.spacing() * instance_index,
              direction.y * params.spacing() * instance_index,
              direction.z * params.spacing() * instance_index,
          }));
        }

        for (const auto& source_shape : source_shapes) {
          add_instance(builder, compound, source_shape, trsf, instance_index == 0);
        }
      }
      break;
    }
    case cccad::geometry::v1::BuildPatternRequest::kCircular: {
      const auto& params = request.circular();
      validate_count(params.count());

      if (!std::isfinite(params.angle_rad())) {
        throw std::runtime_error("circular pattern angle_rad must be finite");
      }

      const auto origin = from_proto(params.axis().origin());
      const Vec3d direction = normalize(from_proto(params.axis().direction()), "circular pattern axis direction");
      const gp_Ax1 axis(gp_Pnt(origin.x, origin.y, origin.z), to_gp_dir(direction));

      for (int instance_index = 0; instance_index < params.count(); ++instance_index) {
        gp_Trsf trsf;
        if (instance_index != 0) {
          trsf.SetRotation(axis, params.angle_rad() * instance_index);
        }

        for (const auto& source_shape : source_shapes) {
          add_instance(builder, compound, source_shape, trsf, instance_index == 0);
        }
      }
      break;
    }
    case cccad::geometry::v1::BuildPatternRequest::PARAMETERS_NOT_SET:
    default:
      throw std::runtime_error("pattern parameters must be linear or circular");
  }

  return compound;
}

} // namespace cccad::geometry
