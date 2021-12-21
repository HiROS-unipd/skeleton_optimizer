// Standard dependencies
#include <fstream>

// Ceres dependencies
#include "ceres/ceres.h"

// ROS dependencies
#include <ros/ros.h>

// Custom External Packages dependencies
#include "skeletons/types.h"
#include "skeletons/utils.h"

// Internal dependencies
#include "skeleton_optimizer/CostFunction.h"
#include "skeleton_optimizer/Optimizer.h"

void hiros::optimizer::Optimizer::configure()
{
  ROS_INFO_STREAM("Hi-ROS Skeleton Optimizer... Configuring");

  if (m_configured) {
    m_configured = false;
    stop();
  }

  m_nh.getParam("node_name", m_params.node_name);

  m_nh.getParam("input_topic_name", m_params.input_topic_name);
  m_nh.getParam("output_topic_name", m_params.output_topic_name);

  m_nh.getParam("number_of_frames_for_calibration", m_params.number_of_frames_for_calibration);
  m_nh.getParam("max_calibration_coefficient_of_variation", m_params.max_calibration_coefficient_of_variation);
  m_nh.getParam("outlier_threshold", m_params.outlier_threshold);

  m_nh.getParam("export_calibration", m_params.export_calibration);
  m_nh.getParam("load_calibration", m_params.load_calibration);
  m_nh.getParam("calibration_file", m_params.calibration_file);

  if (m_params.load_calibration) {
    XmlRpc::XmlRpcValue calib_xml;

    if (!m_nh.getParam("tracks", calib_xml)) {
      ROS_WARN_STREAM("Hi-ROS Skeleton Optimizer Warning: calibration file '" << m_params.calibration_file
                                                                              << "' not found");
    }
    else {
      parseXml(calib_xml);
    }
  }

  if (m_params.input_topic_name.empty() || m_params.output_topic_name.empty()) {
    ROS_FATAL_STREAM("Hi-ROS Skeleton Optimizer Error: Required topics configuration not provided");
    ros::shutdown();
  }

  if (m_params.number_of_frames_for_calibration <= 0) {
    ROS_FATAL_STREAM(
      "Hi-ROS Skeleton Optimizer Error: The number of frames to acquire for the calibration must be greater than 0");
    ros::shutdown();
  }

  if (m_params.max_calibration_coefficient_of_variation <= 0.0) {
    ROS_FATAL_STREAM("Hi-ROS Skeleton Optimizer Error: The maximum coefficient of variation to consider a calibration "
                     "valid must be greater than 0");
    ros::shutdown();
  }

  if (m_params.outlier_threshold <= 0.0) {
    ROS_FATAL_STREAM(
      "Hi-ROS Skeleton Optimizer Error: The threshold to consider a joint an outlier must be greater than 0");
    ros::shutdown();
  }

  m_configured = true;

  ROS_INFO_STREAM(BASH_MSG_GREEN << "Hi-ROS Skeleton Optimizer... CONFIGURED" << BASH_MSG_RESET);
}

void hiros::optimizer::Optimizer::start()
{
  ROS_INFO_STREAM("Hi-ROS Skeleton Optimizer... Starting");

  if (!m_configured) {
    configure();
  }

  setupRosTopics();

  ROS_INFO_STREAM(BASH_MSG_GREEN << "Hi-ROS Skeleton Optimizer... RUNNING" << BASH_MSG_RESET);
  ros::spin();
}

void hiros::optimizer::Optimizer::stop()
{
  ROS_INFO_STREAM("Hi-ROS Skeleton Optimizer... Stopping");

  if (m_in_skeleton_group_sub) {
    m_in_skeleton_group_sub.shutdown();
  }

  if (m_out_msg_pub) {
    m_out_msg_pub.shutdown();
  }

  if (m_calibrate_srv) {
    m_calibrate_srv.shutdown();
  }

  if (m_chande_id_srv) {
    m_chande_id_srv.shutdown();
  }

  ROS_INFO_STREAM(BASH_MSG_GREEN << "Hi-ROS Skeleton Optimizer... STOPPED" << BASH_MSG_RESET);
  ros::shutdown();
}

void hiros::optimizer::Optimizer::parseXml(const XmlRpc::XmlRpcValue& t_xml)
{
  for (int track_idx = 0; track_idx < t_xml.size(); ++track_idx) {
    checkXmlRpcParam("track_id", t_xml[track_idx], XmlRpc::XmlRpcValue::TypeInt);
    checkXmlRpcParam("links", t_xml[track_idx], XmlRpc::XmlRpcValue::TypeArray);

    int track_id = static_cast<int>(t_xml[track_idx]["track_id"]);
    for (int link_idx = 0; link_idx < t_xml[track_idx]["links"].size(); ++link_idx) {
      checkXmlRpcParam("link_id", t_xml[track_idx]["links"][link_idx], XmlRpc::XmlRpcValue::TypeInt);
      checkXmlRpcParam("link_confidence", t_xml[track_idx]["links"][link_idx], XmlRpc::XmlRpcValue::TypeDouble);
      checkXmlRpcParam("link_length", t_xml[track_idx]["links"][link_idx], XmlRpc::XmlRpcValue::TypeDouble);

      int link_id = static_cast<int>(t_xml[track_idx]["links"][link_idx]["link_id"]);
      double link_confidence = static_cast<double>(t_xml[track_idx]["links"][link_idx]["link_confidence"]);
      double link_length = static_cast<double>(t_xml[track_idx]["links"][link_idx]["link_length"]);

      m_calibrated_links[track_id][link_id] = {link_confidence, link_length};
    }
  }
}

void hiros::optimizer::Optimizer::checkXmlRpcParam(const std::string& t_tag,
                                                   const XmlRpc::XmlRpcValue& t_node,
                                                   const XmlRpc::XmlRpcValue::Type t_type) const
{
  if (!checkXmlRpcSanity(t_tag, t_node, t_type)) {
    ROS_FATAL_STREAM("Hi-ROS Skeleton Optimizer Error: " << t_tag << " not found");
    ros::shutdown();
    exit(EXIT_FAILURE);
  }
}

bool hiros::optimizer::Optimizer::checkXmlRpcSanity(const std::string& t_tag,
                                                    const XmlRpc::XmlRpcValue& t_node,
                                                    const XmlRpc::XmlRpcValue::Type t_type) const
{
  if (!t_node.hasMember(t_tag)) {
    std::cerr << "Tag: " << t_tag << ". Not found" << std::endl;
    return false;
  }
  if (t_node[t_tag].getType() != t_type) {
    std::cerr << "Tag: " << t_tag << ". Type different from expected" << std::endl;
    return false;
  }
  if (!t_node[t_tag].valid()) {
    std::cerr << "Tag: " << t_tag << ". Empty value not allowed." << std::endl;
    return false;
  }

  return true;
}

void hiros::optimizer::Optimizer::setupRosTopics()
{
  m_in_skeleton_group_sub = m_nh.subscribe(m_params.input_topic_name, 1, &Optimizer::callback, this);

  while (m_in_skeleton_group_sub.getNumPublishers() == 0 && !ros::isShuttingDown()) {
    ROS_WARN_STREAM_DELAYED_THROTTLE(2, "Hi-ROS Skeleton Optimizer Warning: No input messages on skeleton group topic");
  }

  m_calibrate_srv = m_nh.advertiseService("calibrate", &Optimizer::calibrate, this);
  m_chande_id_srv = m_nh.advertiseService("change_id", &Optimizer::changeId, this);

  m_out_msg_pub = m_nh.advertise<hiros_skeleton_msgs::SkeletonGroup>(m_params.output_topic_name, 1);
}

bool hiros::optimizer::Optimizer::changeId(hiros_skeleton_optimizer::ChangeId::Request& t_req,
                                           hiros_skeleton_optimizer::ChangeId::Response& t_res)
{
  int from_id = t_req.from;
  int to_id = t_req.to;

  if (to_id > from_id) {
    ROS_WARN_STREAM("Hi-ROS Skeleton Optimizer Warning: Track IDs can only be lowered. Cannot change");
    return t_res.ok = false;
  }

  if (std::find_if(
        m_tracks.skeletons.begin(), m_tracks.skeletons.end(), [&](const auto& sk) { return sk.id == from_id; })
      == m_tracks.skeletons.end()) {
    ROS_WARN_STREAM("Hi-ROS Skeleton Optimizer Warning: Track ID " << from_id << " does not exist. Cannot change");
    return t_res.ok = false;
  }

  if (std::find_if(m_tracks.skeletons.begin(), m_tracks.skeletons.end(), [&](const auto& sk) { return sk.id == to_id; })
      != m_tracks.skeletons.end()) {
    ROS_WARN_STREAM("Hi-ROS Skeleton Optimizer Warning: Track ID " << to_id << " already exists. Cannot change");
    return t_res.ok = false;
  }

  ROS_INFO_STREAM("Hi-ROS Skeleton Optimizer... Change ID from " << from_id << " to " << to_id);

  auto it =
    std::find_if(m_ids_to_change.begin(), m_ids_to_change.end(), [&](const auto& p) { return p.second == from_id; });

  // If track with ID from_id had already been changed
  if (it != m_ids_to_change.end()) {
    it->second = to_id;
  }
  // If track with ID from_id is being changed for the first time
  else {
    m_ids_to_change[from_id] = to_id;
  }

  return t_res.ok = true;
}

bool hiros::optimizer::Optimizer::calibrate(hiros_skeleton_optimizer::Calibrate::Request& t_req,
                                            hiros_skeleton_optimizer::Calibrate::Response& t_res)
{
  if (m_acquire_calibration_tracks) {
    ROS_WARN_STREAM("Hi-ROS Skeleton Optimizer Warning: Cannot start a calibration before finishing the previous one");
    return t_res.ok = false;
  }

  m_ids_to_calibrate.clear();

  for (const auto& id : t_req.track_ids) {
    if (std::find_if(m_tracks.skeletons.begin(), m_tracks.skeletons.end(), [&](const auto& sk) { return sk.id == id; })
        == m_tracks.skeletons.end()) {
      ROS_WARN_STREAM("Hi-ROS Skeleton Optimizer Warning: Track ID " << id << " does not exist. Cannot calibrate");
    }
    else {
      m_ids_to_calibrate.push_back(id);
    }
  }

  if (m_ids_to_calibrate.empty()) {
    return t_res.ok = false;
  }

  std::string tracks_to_calibrate_str;
  for (const auto& id : m_ids_to_calibrate) {
    tracks_to_calibrate_str += " " + std::to_string(id);
  }
  ROS_INFO_STREAM("Hi-ROS Skeleton Optimizer... Starting to calibrate tracks:" << tracks_to_calibrate_str);

  m_acquire_calibration_tracks = true;
  return t_res.ok = true;
}

void hiros::optimizer::Optimizer::callback(const hiros_skeleton_msgs::SkeletonGroup& t_msg)
{
  if (!ros::ok()) {
    stop();
    exit(EXIT_FAILURE);
  }

  m_tracks = hiros::skeletons::utils::toStruct(t_msg);

  changeIds();

  if (m_acquire_calibration_tracks) {
    pushTracksToCalibrationBuffer();

    if (m_start_calibration) {
      calibrate();
      exportCalibration();
    }
  }

  fixOutliers();
  optimize();

  m_prev_tracks = m_tracks;

  m_out_msg_pub.publish(hiros::skeletons::utils::toMsg(m_tracks));
}

void hiros::optimizer::Optimizer::changeIds()
{
  for (auto& sk : m_tracks.skeletons) {
    if (m_ids_to_change.find(sk.id) != m_ids_to_change.end()) {
      sk.id = m_ids_to_change.at(sk.id);
    }
  }
}

void hiros::optimizer::Optimizer::pushTracksToCalibrationBuffer()
{
  for (const auto& sk : m_tracks.skeletons) {
    if (std::find(m_ids_to_calibrate.begin(), m_ids_to_calibrate.end(), sk.id) != m_ids_to_calibrate.end()) {
      m_calibration_buffer[sk.id].push_back(sk);
    }
  }

  unsigned long n_acquired_frames =
    std::min_element(m_calibration_buffer.begin(), m_calibration_buffer.end(), [](const auto& s1, const auto& s2) {
      return s1.second.size() < s2.second.size();
    })->second.size();

  if (n_acquired_frames >= static_cast<unsigned long>(m_params.number_of_frames_for_calibration)) {
    m_start_calibration = true;
  }
}

void hiros::optimizer::Optimizer::calibrate()
{
  bool calibration_failed;
  double coeff_of_variation;

  for (const auto& track : m_calibration_buffer) {
    auto link_lengths = computeLinkLengths(track.second);

    calibration_failed = false;

    for (const auto& link_length : link_lengths) {
      auto avg_link = computeAvg(coeff_of_variation, link_length.second);

      if (coeff_of_variation > m_params.max_calibration_coefficient_of_variation) {
        m_calibrated_links.erase(track.first);
        calibration_failed = true;
        ROS_WARN_STREAM("Hi-ROS Skeleton Optimizer Warning: Calibration of track ID " << track.first
                                                                                      << " failed. Repeat");
        break;
      }

      m_calibrated_links[track.first][link_length.first] = avg_link;

      if (calibration_failed) {
        break;
      }
    }
  }

  m_calibration_buffer.clear();
  m_start_calibration = false;
  m_acquire_calibration_tracks = false;
}

void hiros::optimizer::Optimizer::exportCalibration() const
{
  if (!m_params.export_calibration) {
    return;
  }

  std::ofstream file;

  file.open(m_params.calibration_file, std::ios_base::out);
  file << "tracks:" << std::endl;

  for (const auto& track : m_calibrated_links) {
    file << "  - track_id: " << track.first << std::endl;
    file << "    links:" << std::endl;

    for (const auto& link : track.second) {
      file << "    - link_id: " << link.first << std::endl;
      file << "      link_confidence: " << link.second.confidence << std::endl;
      file << "      link_length: " << link.second.length << std::endl;
    }
  }

  file.close();
}

std::map<int, std::vector<hiros::optimizer::LinkInfo>>
hiros::optimizer::Optimizer::computeLinkLengths(const std::vector<hiros::skeletons::types::Skeleton>& t_skeletons) const
{
  std::map<int, std::vector<hiros::optimizer::LinkInfo>> link_lengths;

  for (const auto& skeleton : t_skeletons) {
    for (const auto& link : skeleton.links) {
      auto length = skeletons::utils::linkLength(skeleton, link.id);

      if (!std::isnan(length)) {
        link_lengths[link.id].push_back({link.confidence, length});
      }
    }
  }

  return link_lengths;
}

hiros::optimizer::LinkInfo
hiros::optimizer::Optimizer::computeAvg(double& t_coeff_of_variation,
                                        const std::vector<hiros::optimizer::LinkInfo>& t_links) const
{
  double length_sum = std::accumulate(t_links.begin(), t_links.end(), 0.0, [](double sum, const auto& link) {
    return sum + (link.confidence * link.length);
  });

  double length_square_sum = std::accumulate(t_links.begin(), t_links.end(), 0.0, [](double sum, const auto& link) {
    return sum + (link.confidence * std::pow(link.length, 2));
  });

  double confidence_sum = std::accumulate(
    t_links.begin(), t_links.end(), 0.0, [](double sum, const auto& link) { return sum + link.confidence; });

  double mean_length = length_sum / confidence_sum;
  double stdev_length = std::sqrt(length_square_sum / confidence_sum - mean_length * mean_length);
  double mean_conf = confidence_sum / t_links.size();

  t_coeff_of_variation = stdev_length / mean_length;
  LinkInfo res{mean_conf, mean_length};
  return res;
}

void hiros::optimizer::Optimizer::fixOutliers()
{
  if (m_prev_tracks.skeletons.empty()) {
    return;
  }

  for (auto& track : m_tracks.skeletons) {
    if (m_calibrated_links.count(track.id) > 0) {
      for (const auto& link_pair : m_calibrated_links.at(track.id)) {
        if (track.hasLink(link_pair.first)) {
          auto& link = track.getLink(link_pair.first);
          auto parent_marker_id = link.parent_marker;
          auto child_marker_id = link.child_marker;

          if (track.hasMarker(parent_marker_id) && track.hasMarker(child_marker_id)) {
            fixOutlier(track, link_pair.first, link_pair.second.length);
          }
        }
      }
    }
  }
}

void hiros::optimizer::Optimizer::fixOutlier(hiros::skeletons::types::Skeleton& t_track,
                                             const int& t_link_id,
                                             const double& t_link_length)
{
  if (!t_track.hasLink(t_link_id)) {
    return;
  }

  auto& link = t_track.getLink(t_link_id);

  double length_error = std::abs(skeletons::utils::linkLength(t_track, t_link_id) - t_link_length) / t_link_length;

  if (length_error > m_params.outlier_threshold) {
    if (m_prev_tracks.hasSkeleton(t_track.id)) {
      auto& prev_track = m_prev_tracks.getSkeleton(t_track.id);

      if (prev_track.hasMarker(link.child_marker)) {
        if (t_track.hasMarker(link.child_marker)) {
          t_track.getMarker(link.child_marker) = prev_track.getMarker(link.child_marker);
          return;
        }
      }
    }

    t_track.removeMarker(link.child_marker);
  }
}

void hiros::optimizer::Optimizer::optimize()
{
  for (auto& track : m_tracks.skeletons) {
    if (hasCalibration(track.id)) {
      ceres::Problem problem;
      ceres::Solver::Options options;
      options.logging_type = ceres::LoggingType::SILENT;
      ceres::Solver::Summary summary;

      for (const auto& link : track.links) {
        if (track.hasMarker(link.parent_marker) && track.hasMarker(link.child_marker)) {
          auto& parent_marker = track.getMarker(link.parent_marker);
          auto& child_marker = track.getMarker(link.child_marker);

          ceres::CostFunction* cost_function =
            new ceres::AutoDiffCostFunction<CostFunction, 1, 1, 1, 1, 1, 1, 1>(new CostFunction(
              m_params.outlier_threshold, m_calibrated_links.at(track.id).at(link.id).length, track, link));

          problem.AddResidualBlock(cost_function,
                                   nullptr,
                                   const_cast<double*>(&parent_marker.center.pose.position.x()),
                                   const_cast<double*>(&parent_marker.center.pose.position.y()),
                                   const_cast<double*>(&parent_marker.center.pose.position.z()),
                                   const_cast<double*>(&child_marker.center.pose.position.x()),
                                   const_cast<double*>(&child_marker.center.pose.position.y()),
                                   const_cast<double*>(&child_marker.center.pose.position.z()));
        }
      }

      ceres::Solve(options, &problem, &summary);
    }
  }
}

bool hiros::optimizer::Optimizer::hasCalibration(const int& t_track_id) const
{
  return m_calibrated_links.find(t_track_id) != m_calibrated_links.end();
}
