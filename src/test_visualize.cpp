/// @file
///
/// This file creates a simple trajectory and visualize it.

/* Examples

PublishFramesToLcm("DRAKE_DRAW_TRAJECTORY", {
    {"X_WF", Eigen::Isometry3d::Identity()},
    {"X_WG", Eigen::Isometry3d::Identity()},
   }, &lcm);
*/

#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
//#include "drake/common/text_logging_gflags.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_viewer_draw.hpp"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace examples {
namespace eve {

void PublishFramesToLcm(const std::string& channel_name,
                        const std::vector<Eigen::Isometry3d>& poses,
                        const std::vector<std::string>& names,
                        drake::lcm::DrakeLcmInterface* dlcm) {
  DRAKE_DEMAND(poses.size() == names.size());
  drake::lcmt_viewer_draw frame_msg{};
  frame_msg.timestamp = 0;
  int32_t vsize = poses.size();
  frame_msg.num_links = vsize;
  frame_msg.link_name.resize(vsize);
  frame_msg.robot_num.resize(vsize, 0);

  for (size_t i = 0; i < poses.size(); i++) {
    Eigen::Isometry3f pose = poses[i].cast<float>();
    // Create a frame publisher
    Eigen::Vector3f goal_pos = pose.translation();
    Eigen::Quaternion<float> goal_quat =
        Eigen::Quaternion<float>(pose.linear());
    frame_msg.link_name[i] = names[i];
    frame_msg.position.push_back({goal_pos(0), goal_pos(1), goal_pos(2)});
    frame_msg.quaternion.push_back(
        {goal_quat.w(), goal_quat.x(), goal_quat.y(), goal_quat.z()});
  }

  const int num_bytes = frame_msg.getEncodedSize();
  const size_t size_bytes = static_cast<size_t>(num_bytes);
  std::vector<uint8_t> bytes(size_bytes);
  frame_msg.encode(bytes.data(), 0, num_bytes);
  dlcm->Publish("DRAKE_DRAW_TRAJECTORY_" + channel_name, bytes.data(),
                num_bytes, {});
}

void PublishFramesToLcm(
    const std::string& channel_name,
    const std::unordered_map<std::string, Eigen::Isometry3d>& name_to_frame_map,
    drake::lcm::DrakeLcmInterface* lcm) {
  std::vector<Eigen::Isometry3d> poses;
  std::vector<std::string> names;
  for (const auto& pair : name_to_frame_map) {
    poses.push_back(pair.second);
    names.push_back(pair.first);
  }
  PublishFramesToLcm(channel_name, poses, names, lcm);
}

void DoMain() {
  // Design the trajectory to follow.
  const std::vector<double> kTimes{0.0, 2.0, 5.0, 10.0};
  std::vector<Eigen::MatrixXd> knots(kTimes.size());
  Eigen::VectorXd tmp1(3);
  tmp1 << 0, 0, 0;
  knots[0] = tmp1;
  Eigen::VectorXd tmp2(3);
  tmp2 << 1, 1, 0;
  knots[1] = tmp2;
  Eigen::VectorXd tmp3(3);
  tmp3 << 2, -1, 0;
  knots[2] = tmp3;
  Eigen::VectorXd tmp4(3);
  tmp4 << 3, 0, 0;
  knots[3] = tmp4;
  Eigen::VectorXd knot_dot_start = Eigen::VectorXd::Zero(3);
  Eigen::MatrixXd knot_dot_end = Eigen::VectorXd::Zero(3);
  trajectories::PiecewisePolynomial<double> trajectory =
      trajectories::PiecewisePolynomial<double>::Cubic(
          kTimes, knots, knot_dot_start, knot_dot_end);

  std::vector<std::string> names;
  std::vector<Eigen::Isometry3d> poses;
  for (double t = 0.0; t < 10.0; t += 0.1) {
    names.push_back("X" + std::to_string(int(t * 100)));
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = trajectory.value(t);
    poses.push_back(pose);
  }
  lcm::DrakeLcm lcm;

  //  std::vector<std::string> names = {"X_WF", "X_WG"};
  //  Eigen::Isometry3d pose1 = Eigen::Isometry3d::Identity();
  //  pose1.translation() = Eigen::Vector3d::Ones()*0.5;
  //  Eigen::Isometry3d pose2 = Eigen::Isometry3d::Identity();
  //  Eigen::Vector3d translation2; translation2 << 1,2,3;
  //  pose1.translation() = translation2;
  //  std::vector<Eigen::Isometry3d> poses = {pose1, pose2};

  PublishFramesToLcm("DRAKE_DRAW_TRAJECTORY", poses, names, &lcm);
}
}  // namespace eve
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "A simple dynamic simulation for the Allegro hand moving under constant"
      " torques.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::examples::eve::DoMain();
  return 0;
}
