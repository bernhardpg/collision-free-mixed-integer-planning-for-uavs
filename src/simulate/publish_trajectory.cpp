#include "simulate/publish_trajectory.h"

// Taken from tutorial: https://drake.guzhaoyuan.com/thing-to-do-in-drake/visualize-trajectory-in-drake-visualizer?fbclid=IwAR1KQynSN3pgmxLXAbBS0Snt1lhv3RviYzERoPqMKZo6c_04GxJvzyTG1qo
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
