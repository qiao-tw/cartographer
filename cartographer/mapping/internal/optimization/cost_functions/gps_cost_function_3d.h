#ifndef CARTOGRAPHER_MAPPING_INTERNAL_OPTIMIZATION_COST_FUNCTIONS_GPS_COST_FUNCTION_3D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_OPTIMIZATION_COST_FUNCTIONS_GPS_COST_FUNCTION_3D_H_

#include <array>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/common/math.h"
#include "cartographer/mapping/internal/optimization/cost_functions/cost_helpers.h"
#include "cartographer/mapping/pose_graph.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "ceres/ceres.h"
#include "ceres/jet.h"

namespace cartographer {
namespace mapping {
namespace optimization {

class GpsCostFunction3D {
 public:
  static ceres::CostFunction* CreateAutoDiffCostFunction(
      const PoseGraph::ConstraintFixedFrame::PoseFixedFrame& pose) {
    return new ceres::AutoDiffCostFunction<GpsCostFunction3D, 6 /* residuals */,
                                           4 /* rotation variables */,
                                           3 /* translation variables */>(
        new GpsCostFunction3D(pose));
  }

  template <typename T>
  bool operator()(const T* const global_rotation,
                  const T* const global_translation, T* const e) const {
    const std::array<T, 6> error =
        ScaleError(ComputeFixedFrameError(pose_.zbar_ij, global_rotation,
                                          global_translation),
                   pose_.translation_xy_weight, pose_.translation_z_weight,
                   pose_.rotation_yaw_weight, pose_.rotation_roll_pitch_weight);
    std::copy(std::begin(error), std::end(error), e);
    return true;
  }

 private:
  explicit GpsCostFunction3D(
      const PoseGraph::ConstraintFixedFrame::PoseFixedFrame& pose)
      : pose_(pose) {}

  const PoseGraph::ConstraintFixedFrame::PoseFixedFrame pose_;
};

class GpsSubmapCostFunction3D {
 public:
  static ceres::CostFunction* CreateAutoDiffCostFunction(
      const PoseGraph::ConstraintFixedFrame::PoseFixedFrame& constraint_pose,
      const transform::Rigid3d& relative_pose) {
    return new ceres::AutoDiffCostFunction<
        GpsSubmapCostFunction3D, 6 /* residuals */, 4 /* rotation variables */,
        3 /* translation variables */>(
        new GpsSubmapCostFunction3D(constraint_pose, relative_pose));
  }

  template <typename T>
  bool operator()(const T* const submap_rotation,
                  const T* const submap_translation, T* const e) const {
    const std::array<T, 6> error = ScaleError(
        ComputeFixedFrameError(constraint_pose_.zbar_ij, relative_pose_,
                               submap_rotation, submap_translation),
        constraint_pose_.translation_xy_weight,
        constraint_pose_.translation_z_weight,
        constraint_pose_.rotation_yaw_weight,
        constraint_pose_.rotation_roll_pitch_weight);
    std::copy(std::begin(error), std::end(error), e);
    return true;
  }

 private:
  explicit GpsSubmapCostFunction3D(
      const PoseGraph::ConstraintFixedFrame::PoseFixedFrame& constraint_pose,
      const transform::Rigid3d& relative_pose)
      : constraint_pose_(constraint_pose), relative_pose_(relative_pose) {}

  const PoseGraph::ConstraintFixedFrame::PoseFixedFrame constraint_pose_;
  const transform::Rigid3d relative_pose_;
};

}  // namespace optimization
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_OPTIMIZATION_COST_FUNCTIONS_GPS_COST_FUNCTION_3D_H_
