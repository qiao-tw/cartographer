/*
 * Copyright 2018 Geometrical PAL
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

#include "cartographer/io/internal/pbstream_export.h"

#include <map>
#include <sstream>
#include <string>

#include "cartographer/io/proto_stream.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/mapping/proto/trajectory.pb.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

namespace cartographer {
namespace io {
namespace {

void Run(const std::string& pbstream_filename, const std::string& output_filename) {
  LOG(INFO) << "Reading pbstream file from '" << pbstream_filename << "'...";
  io::ProtoStreamReader reader(pbstream_filename);
  io::ProtoStreamDeserializer deserializer(&reader);
  const auto header = deserializer.header();
  LOG(INFO) << "Header: " << header.DebugString();
  for (const mapping::proto::TrajectoryBuilderOptionsWithSensorIds&
           trajectory_options : deserializer.all_trajectory_builder_options()
                                    .options_with_sensor_ids()) {
    LOG(INFO) << "Trajectory options: " << trajectory_options.DebugString();
  }
  const mapping::proto::PoseGraph pose_graph = deserializer.pose_graph();
  std::ofstream ofile(output_filename);
  for (const mapping::proto::Trajectory& trajectory : pose_graph.trajectory()) {
    LOG(INFO) << "Trajectory id: " << trajectory.trajectory_id()
              << " has #nodes " << trajectory.node_size() << " has #submaps "
              << trajectory.submap_size();
    // qiao@20181003: display all nodes in this trajectory
    for (const mapping::proto::Trajectory_Node& node : trajectory.node()) {
      LOG(INFO) << "Node id: " << node.node_index() << "\n"
                << "timestamp: " << node.timestamp() << "\n"
                << "pose:\n" << node.pose().DebugString() << "\n";

      // qiao@20181003
      // FIXME: use json file format
      ofile << node.node_index() << " "
            << node.timestamp() << " "
            << node.pose().translation().x() << " "
            << node.pose().translation().y() << " "
            << node.pose().translation().z() << " "
            << node.pose().rotation().w() << " "
            << node.pose().rotation().x() << " "
            << node.pose().rotation().y() << " "
            << node.pose().rotation().z() << " "
            << node.pose().translation().x() << " "
            << node.pose().translation().y() << " "
            << node.pose().translation().z() << " "
            << node.pose().rotation().w() << " "
            << node.pose().rotation().x() << " "
            << node.pose().rotation().y() << " "
            << node.pose().rotation().z() << std::endl;
    }
  }
}
}  // namespace

int pbstream_export(int argc, char* argv[]) {
  std::stringstream ss;
  ss << "\n\n"
     << "Reads a pbstream file and export its contents to a json file.\n\n"
     << "Usage: " << argv[0] << " " << argv[1]
     << " <pbstream_filename> <output_filename>\n";
  google::SetUsageMessage(ss.str());
  if (argc != 4) {
    google::ShowUsageWithFlagsRestrict(argv[0], "pbstream_export");
    return EXIT_FAILURE;
  }
  Run(argv[2], argv[3]);
  return EXIT_SUCCESS;
}

}  // namespace io
}  // namespace cartographer
