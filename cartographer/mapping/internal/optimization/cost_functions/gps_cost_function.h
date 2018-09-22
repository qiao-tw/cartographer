/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_OPTIMIZATION_COST_FUNCTIONS_GPS_COST_FUNCTION_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_OPTIMIZATION_COST_FUNCTIONS_GPS_COST_FUNCTION_H_

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

class GpsCostFunction {
 public:
  static ceres::CostFunction* CreateAutoDiffCostFunction(
      const PoseGraph::Constraint::Pose& pose) {
    return new ceres::AutoDiffCostFunction<
        GpsCostFunction, 3 /* residuals */, 4 /* rotation variables */,
        3 /* translation variables */, 4 /* rotation variables */>(new GpsCostFunction(pose));
  }
  explicit GpsCostFunction(
      const PoseGraph::Constraint::Pose& observed_relative_pose)
      : observed_relative_pose_(observed_relative_pose) {}

  template <typename T>
  bool operator()(const T* const c_rotation, const T* const c_translation,
                  const T* const gps_rotation,
                  T* const e) const {
    ComputeScaledError_(observed_relative_pose_, c_rotation, c_translation, gps_rotation, e);
    return true;
  }

 private:
  const PoseGraph::Constraint::Pose observed_relative_pose_;
  // Computes the error between the node-to-gps alignment 'zbar_ij' and the
  // difference of node pose 'c_i' and gps pose 'c_j' which are both in an
  // arbitrary common frame.
  template <typename T>
  static std::array<T, 3> ComputeUnscaledError_(
      const transform::Rigid3d& z_gps, const T* const c_rotation,
      const T* const c_translation, const T* const gps_rotation) {
    // const Eigen::Quaternion<T> R_gps_inverse(gps_rotation[0], -gps_rotation[1],
    //                                         -gps_rotation[2], -gps_rotation[3]);
    // HACK: only optimize yaw for now.
    const Eigen::Quaternion<T> R_gps_inverse(gps_rotation[0], (T)0, (T)0, -gps_rotation[3]);
    const Eigen::Matrix<T, 3, 1> delta(c_translation[0], c_translation[1], c_translation[2]);
    const Eigen::Matrix<T, 3, 1> h_translation = R_gps_inverse * delta;
    return {{T(z_gps.translation().x()) - h_translation[0],
            T(z_gps.translation().y()) - h_translation[1],
            T(z_gps.translation().z()) - h_translation[2]}};
  }
  
  // Computes the error scaled by 'translation_weight' and 'rotation_weight',
  // storing it in 'e'.
  template <typename T>
  static void ComputeScaledError_(const PoseGraph::Constraint::Pose& pose,
                                 const T* const c_rotation,
                                 const T* const c_translation,
                                 const T* const gps_rotation, T* const e) {
    const std::array<T, 3> e_ij = 
        ComputeUnscaledError_(pose.zbar_ij, c_rotation, c_translation, gps_rotation);
    for (int ij : {0, 1, 2}) {
      e[ij] = e_ij[ij] * T(pose.translation_weight);
    }
  }
}; 

}  // namespace optimization
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_OPTIMIZATION_COST_FUNCTIONS_GPS_COST_FUNCTION_H_
