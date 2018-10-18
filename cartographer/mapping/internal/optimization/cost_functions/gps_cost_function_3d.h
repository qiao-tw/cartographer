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
      const PoseGraph::Constraint::Pose& pose,
      const PoseGraph::Constraint::Pose& relative_pose) {
    return new ceres::AutoDiffCostFunction<GpsCostFunction3D, 3 /* residuals */,
                                           4 /* rotation variables */,
                                           3 /* translation variables */>(
        new GpsCostFunction3D(pose, relative_pose));
  }

  template <typename T>
  bool operator()(const T* const c_ij_rotation, const T* const c_ij_translation,
                  T* const e) const {
    const std::array<T, 3> error = ScaleError(
        ComputeFixedPosistionError(pose_.zbar_ij.translation(),
                                   relative_pose_.zbar_ij.translation(),
                                   c_ij_rotation, c_ij_translation),
        pose_.translation_weight);
    std::copy(std::begin(error), std::end(error), e);
    return true;
  }

 private:
  explicit GpsCostFunction3D(const PoseGraph::Constraint::Pose& pose,
                             const PoseGraph::Constraint::Pose& relative_pose)
      : pose_(pose), relative_pose_(relative_pose) {}

  const PoseGraph::Constraint::Pose pose_;
  const PoseGraph::Constraint::Pose relative_pose_;
};

}  // namespace optimization
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_OPTIMIZATION_COST_FUNCTIONS_GPS_COST_FUNCTION_3D_H_
