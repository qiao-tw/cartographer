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

#include "cartographer/mapping/pose_graph_trimmer.h"

#include "glog/logging.h"

namespace cartographer {
namespace mapping {

PureLocalizationTrimmer::PureLocalizationTrimmer(const int trajectory_id,
                                                 const int num_submaps_to_keep)
    : trajectory_id_(trajectory_id), num_submaps_to_keep_(num_submaps_to_keep) {
  CHECK_GE(num_submaps_to_keep, 2) << "Cannot trim with less than 2 submaps";
}

void PureLocalizationTrimmer::Trim(Trimmable* const pose_graph) {
  if (pose_graph->IsFinished(trajectory_id_)) {
    num_submaps_to_keep_ = 0;
  }

  auto submap_ids = pose_graph->GetSubmapIds(trajectory_id_);
  for (std::size_t i = 0; i + num_submaps_to_keep_ < submap_ids.size(); ++i) {
    pose_graph->TrimSubmap(submap_ids.at(i));
  }

  if (num_submaps_to_keep_ == 0) {
    finished_ = true;
    pose_graph->SetTrajectoryState(
        trajectory_id_, PoseGraphInterface::TrajectoryState::DELETED);
  }
}

bool PureLocalizationTrimmer::IsFinished() { return finished_; }

ItriTrimmer::ItriTrimmer(int trajectory_id,
                         int num_submaps_to_keep)
    : trajectory_id_(trajectory_id),
      num_submaps_to_keep_(num_submaps_to_keep)
{

}

void ItriTrimmer::Trim(Trimmable* const pose_graph)
{
  while ( pose_graph->num_submaps(trajectory_id_) > num_submaps_to_keep_ ) {
    const int submap_index_to_trim_next = num_submaps_trimmed_;
    pose_graph->TrimSubmap(
        SubmapId{trajectory_id_, submap_index_to_trim_next});
    ++num_submaps_trimmed_;
  }

  if ( num_submaps_to_keep_ == 0 ) {
    finished_ = true;
  }
}

bool ItriTrimmer::IsFinished()
{
    return finished_;
}

}  // namespace mapping
}  // namespace cartographer
