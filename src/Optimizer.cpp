// Ceres dependencies
#include "ceres/ceres.h"

// ROS dependencies
#include <ros/ros.h>

// Custom Ros Message dependencies
#include "skeleton_msgs/SkeletonGroup.h"

// Custom External Packages dependencies
#include "skeletons/types.h"
#include "skeletons/utils.h"

// Internal dependencies
#include "skeleton_optimizer/CostFunction.h"
#include "skeleton_optimizer/Optimizer.h"

hiros::optimizer::Optimizer::Optimizer()
  : m_nh("~")
  , m_node_namespace(m_nh.getNamespace())
  , m_acquire_calibration_tracks(false)
  , m_start_calibration(false)
  , m_configured(false)
{}

hiros::optimizer::Optimizer::~Optimizer() {}

void hiros::optimizer::Optimizer::configure()
{
  ROS_INFO_STREAM("Hi-ROS Skeleton Optimizer...Configuring");

  if (m_configured) {
    m_configured = false;
    stop();
  }

  m_nh.getParam("node_name", m_params.node_name);

  m_nh.getParam("input_topic_name", m_params.input_topic_name);
  m_nh.getParam("output_topic_name", m_params.output_topic_name);

  m_nh.getParam("define_links", m_params.define_links);
  m_nh.getParam("number_of_frames_for_calibration", m_params.number_of_frames_for_calibration);
  m_nh.getParam("max_calibration_coefficient_of_variation", m_params.max_calibration_coefficient_of_variation);
  m_nh.getParam("outlier_threshold", m_params.outlier_threshold);

  if (m_params.define_links) {
    XmlRpc::XmlRpcValue xml_links;
    m_nh.getParam("links", xml_links);

    for (int i = 0; i < xml_links.size(); ++i) {
      int skeleton_part_id = xml_links[i]["skeleton_part"];

      std::vector<utils::Link> links;
      for (int j = 0; j < xml_links[i]["links"].size(); ++j) {
        m_links[skeleton_part_id].push_back({xml_links[i]["links"][j]["id"],
                                             xml_links[i]["links"][j]["name"],
                                             xml_links[i]["links"][j]["parent_joint"],
                                             xml_links[i]["links"][j]["child_joint"],
                                             std::numeric_limits<double>::quiet_NaN(),
                                             std::numeric_limits<double>::quiet_NaN()});
      }
    }
  }

  if (m_params.input_topic_name.empty() || m_params.output_topic_name.empty()) {
    ROS_FATAL_STREAM("Required topics configuration not provided. Unable to continue");
    ros::shutdown();
  }

  if (m_params.number_of_frames_for_calibration <= 0) {
    ROS_FATAL_STREAM("The number of frames to acquire for the calibration must be greater than 0. Unable to continue");
    ros::shutdown();
  }

  if (m_params.max_calibration_coefficient_of_variation <= 0.0) {
    ROS_FATAL_STREAM("The maximum coefficient of variation to consider a calibration valid must be greater than 0. "
                     "Unable to continue");
    ros::shutdown();
  }

  if (m_params.outlier_threshold <= 0.0) {
    ROS_FATAL_STREAM("The threshold to consider a joint an outlier must be greater than 0. Unable to continue");
    ros::shutdown();
  }

  m_configured = true;

  ROS_INFO_STREAM(BASH_MSG_GREEN << "Hi-ROS Skeleton Optimizer...CONFIGURED" << BASH_MSG_RESET);
}

void hiros::optimizer::Optimizer::start()
{
  ROS_INFO_STREAM("Hi-ROS Skeleton Optimizer...Starting");

  if (!m_configured) {
    configure();
  }

  setupRosTopics();

  ROS_INFO_STREAM(BASH_MSG_GREEN << "Hi-ROS Skeleton Optimizer...RUNNING" << BASH_MSG_RESET);
}

void hiros::optimizer::Optimizer::stop()
{
  ROS_INFO_STREAM("Hi-ROS Skeleton Optimizer...Stopping");

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

  ROS_INFO_STREAM(BASH_MSG_GREEN << "Hi-ROS Skeleton Optimizer...STOPPED" << BASH_MSG_RESET);
}

std::string hiros::optimizer::Optimizer::extractImageQualityFromTopicName(const std::string& t_topic_name) const
{
  auto tmp1 = t_topic_name.find_last_of("/");
  std::string tmp2 = t_topic_name.substr(0, tmp1);
  auto tmp3 = tmp2.find_last_of("/");

  return t_topic_name.substr(tmp3 + 1, tmp1 - tmp3 - 1);
}

void hiros::optimizer::Optimizer::setupRosTopics()
{
  m_in_skeleton_group_sub = m_nh.subscribe(m_params.input_topic_name, 1, &Optimizer::callback, this);

  while (m_in_skeleton_group_sub.getNumPublishers() == 0 && !ros::isShuttingDown()) {
    ROS_WARN_STREAM_THROTTLE(2, m_node_namespace << " No input messages on skeleton group topic");
  }

  m_calibrate_srv = m_nh.advertiseService("calibrate", &Optimizer::calibrate, this);
  m_chande_id_srv = m_nh.advertiseService("change_id", &Optimizer::changeId, this);

  std::string out_msg_topic = m_params.output_topic_name;
  std::string image_quality = extractImageQualityFromTopicName(m_params.input_topic_name);
  if (!image_quality.empty()) {
    out_msg_topic.insert(0, image_quality + "/");
  }
  m_out_msg_pub = m_nh.advertise<skeleton_msgs::SkeletonGroup>(out_msg_topic, 1);
}

bool hiros::optimizer::Optimizer::changeId(hiros_skeleton_optimizer::ChangeId::Request& t_req,
                                           hiros_skeleton_optimizer::ChangeId::Response& t_res)
{
  int from_id = t_req.from;
  int to_id = t_req.to;

  if (to_id > from_id) {
    ROS_WARN_STREAM("Track IDs can only be lowered. Cannot change");
    return t_res.ok = false;
  }

  if (std::find_if(m_tracks.skeletons.begin(),
                   m_tracks.skeletons.end(),
                   [from_id](const hiros::skeletons::types::Skeleton& sk) { return sk.id == from_id; })
      == m_tracks.skeletons.end()) {
    ROS_WARN_STREAM("Track ID " << from_id << " does not exist. Cannot change");
    return t_res.ok = false;
  }

  if (std::find_if(m_tracks.skeletons.begin(),
                   m_tracks.skeletons.end(),
                   [to_id](const hiros::skeletons::types::Skeleton& sk) { return sk.id == to_id; })
      != m_tracks.skeletons.end()) {
    ROS_WARN_STREAM("Track ID " << to_id << " already exists. Cannot change");
    return t_res.ok = false;
  }

  ROS_INFO_STREAM("Change ID from " << from_id << " to " << to_id);

  auto it = std::find_if(m_ids_to_change.begin(), m_ids_to_change.end(), [from_id](const std::pair<int, int>& p) {
    return p.second == from_id;
  });

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
  if (!m_params.define_links) {
    ROS_FATAL_STREAM("Cannot calibrate a skeleton without defining the links");
    return t_res.ok = false;
  }

  if (m_acquire_calibration_tracks) {
    ROS_INFO_STREAM("Cannot start a calibration before finishing the previous one");
    return t_res.ok = false;
  }

  m_ids_to_calibrate.clear();

  for (const auto& id : t_req.track_ids) {
    if (std::find_if(m_tracks.skeletons.begin(),
                     m_tracks.skeletons.end(),
                     [id](const hiros::skeletons::types::Skeleton& sk) { return sk.id == id; })
        == m_tracks.skeletons.end()) {
      ROS_WARN_STREAM("Track ID " << id << " does not exist. Cannot calibrate");
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
  ROS_INFO_STREAM("Starting to calibrate tracks: " << tracks_to_calibrate_str);

  m_acquire_calibration_tracks = true;

  return t_res.ok = true;
}

void hiros::optimizer::Optimizer::callback(skeleton_msgs::SkeletonGroupConstPtr t_skeleton_group_msg)
{
  auto start = ros::Time::now();

  m_tracks = hiros::skeletons::utils::toStruct(*t_skeleton_group_msg.get());

  changeIds();

  if (m_acquire_calibration_tracks) {
    pushTracksToCalibrationBuffer();

    if (m_start_calibration) {
      calibrate();
    }
  }

  fixOutliers();
  optimize();

  m_prev_tracks = m_tracks;

  m_out_msg_pub.publish(hiros::skeletons::utils::toMsg(ros::Time::now(),
                                                       t_skeleton_group_msg->header.frame_id,
                                                       t_skeleton_group_msg->src_time,
                                                       t_skeleton_group_msg->src_frame,
                                                       m_tracks));

  auto end = ros::Time::now();
  std::cout << "elapsed " << (end - start).toSec() * 1000. << " ms" << std::endl;
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
    std::min_element(m_calibration_buffer.begin(),
                     m_calibration_buffer.end(),
                     [](std::map<int, std::vector<hiros::skeletons::types::Skeleton>>::const_reference s1,
                        std::map<int, std::vector<hiros::skeletons::types::Skeleton>>::const_reference s2) {
                       return s1.second.size() < s2.second.size();
                     })
      ->second.size();

  if (n_acquired_frames >= static_cast<unsigned long>(m_params.number_of_frames_for_calibration)) {
    m_start_calibration = true;
  }
}

void hiros::optimizer::Optimizer::calibrate()
{
  bool calibration_failed;
  double coeff_of_variation;

  for (const auto& track : m_calibration_buffer) {
    computeLinkLenghts(track.second);

    calibration_failed = false;

    for (const auto& skp : m_link_lengths_vector.at(track.first)) {
      for (const auto& link_vec : skp.second) {
        utils::Link avg_link = computeAvgLink(coeff_of_variation, link_vec.second);

        std::cout << "id: " << track.first << " skp: " << skp.first << " link: " << avg_link.id
                  << "\tl: " << avg_link.length << "\t+- " << coeff_of_variation * avg_link.length
                  << "\tm conf: " << avg_link.confidence << " (" << avg_link.name << ")" << std::endl;

        if (coeff_of_variation > m_params.max_calibration_coefficient_of_variation) {
          m_calibrated_links[track.first].clear();
          calibration_failed = true;
          ROS_WARN_STREAM("Calibration of track ID " << track.first << " failed. Repeat");
          break;
        }

        m_calibrated_links[track.first][skp.first][avg_link.id] = avg_link;
      }

      if (calibration_failed) {
        break;
      }
    }
  }

  m_calibration_buffer.clear();
  m_start_calibration = false;
  m_acquire_calibration_tracks = false;
}

void hiros::optimizer::Optimizer::computeLinkLenghts(const std::vector<hiros::skeletons::types::Skeleton>& t_skeletons)
{
  for (const auto& sk : t_skeletons) {
    for (const auto& skp : sk.skeleton_parts) {
      if (m_links.find(skp.first) != m_links.end()) {
        for (const auto& link : m_links.at(skp.first)) {
          if (hiros::skeletons::utils::hasKeypoint(skp.second, link.parent_joint)
              && hiros::skeletons::utils::hasKeypoint(skp.second, link.child_joint)) {
            m_link_lengths_vector[sk.id][skp.first][link.id].push_back(computeLinkLength(link, skp.first, sk));
          }
        }
      }
    }
  }
}

hiros::optimizer::utils::Link
hiros::optimizer::Optimizer::computeLinkLength(const hiros::optimizer::utils::Link& t_link,
                                               const int& t_skeleton_part_id,
                                               const hiros::skeletons::types::Skeleton& t_skeleton) const
{
  utils::Link res = t_link;
  res.length = hiros::skeletons::utils::distance(
    t_skeleton.skeleton_parts.at(t_skeleton_part_id).keypoints.at(t_link.parent_joint).point.position,
    t_skeleton.skeleton_parts.at(t_skeleton_part_id).keypoints.at(t_link.child_joint).point.position);
  res.confidence =
    std::min(t_skeleton.skeleton_parts.at(t_skeleton_part_id).keypoints.at(t_link.parent_joint).confidence,
             t_skeleton.skeleton_parts.at(t_skeleton_part_id).keypoints.at(t_link.child_joint).confidence);
  return res;
}

void hiros::optimizer::Optimizer::fixOutliers()
{
  if (m_prev_tracks.skeletons.empty()) {
    return;
  }

  auto tmp_tracks = m_tracks;

  for (const auto& track : tmp_tracks.skeletons) {
    if (m_calibrated_links.find(track.id) != m_calibrated_links.end()) {
      for (const auto& skp : track.skeleton_parts) {
        if (m_calibrated_links.at(track.id).find(skp.first) != m_calibrated_links.at(track.id).end()) {
          for (const auto& link : m_calibrated_links.at(track.id).at(skp.first)) {
            if (hiros::skeletons::utils::hasKeypoint(skp.second, link.second.parent_joint)
                && hiros::skeletons::utils::hasKeypoint(skp.second, link.second.child_joint)) {
              fixOutlier(link.second, skp.first, track);
            }
          }
        }
      }
    }
  }
}

void hiros::optimizer::Optimizer::fixOutlier(const hiros::optimizer::utils::Link& t_link,
                                             const int& t_skeleton_part_id,
                                             const hiros::skeletons::types::Skeleton& t_track)
{

  auto prev_track = hiros::skeletons::utils::getSkeleton(m_prev_tracks, t_track.id);
  auto curr_track = hiros::skeletons::utils::getSkeleton(m_tracks, t_track.id);

  double length_error =
    std::abs(hiros::skeletons::utils::distance(
               t_track.skeleton_parts.at(t_skeleton_part_id).keypoints.at(t_link.parent_joint).point.position,
               t_track.skeleton_parts.at(t_skeleton_part_id).keypoints.at(t_link.child_joint).point.position)
             - t_link.length)
    / t_link.length;

  if (length_error > m_params.outlier_threshold) {
    if (prev_track != nullptr
        && hiros::skeletons::utils::hasKeypoint(*prev_track.get(), t_skeleton_part_id, t_link.child_joint)) {
      if (hiros::skeletons::utils::hasKeypoint(*curr_track.get(), t_skeleton_part_id, t_link.child_joint)) {
        curr_track->skeleton_parts.at(t_skeleton_part_id).keypoints.at(t_link.child_joint) =
          prev_track->skeleton_parts.at(t_skeleton_part_id).keypoints.at(t_link.child_joint);
      }
    }
    else if (hiros::skeletons::utils::hasKeypoint(*curr_track.get(), t_skeleton_part_id, t_link.child_joint)) {
      curr_track->skeleton_parts.at(t_skeleton_part_id).keypoints.erase(t_link.child_joint);
    }
  }
}

void hiros::optimizer::Optimizer::optimize()
{
  for (auto& track : m_tracks.skeletons) {
    if (hasCalibration(track.id)) {
      ceres::Problem problem;
      ceres::Solver::Options options;
      options.logging_type = ceres::LoggingType::SILENT;
      // options.minimizer_progress_to_stdout = true;
      // options.linear_solver_type = ceres::DENSE_QR;
      // options.use_explicit_schur_complement = true;
      // options.max_num_iterations = 10000;
      // options.num_threads = 4;
      ceres::Solver::Summary summary;

      for (auto& skp : track.skeleton_parts) {
        if (m_calibrated_links.at(track.id).find(skp.first) != m_calibrated_links.at(track.id).end()) {
          for (const auto& link : m_links.at(skp.first)) {
            if (hiros::skeletons::utils::hasKeypoint(skp.second, link.parent_joint)
                && hiros::skeletons::utils::hasKeypoint(skp.second, link.child_joint)) {
              ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<CostFunction, 1, 1, 1, 1, 1, 1, 1>(
                new CostFunction(m_params.outlier_threshold,
                                 skp.second.keypoints.at(link.parent_joint).point.position,
                                 skp.second.keypoints.at(link.child_joint).point.position,
                                 m_calibrated_links.at(track.id).at(skp.first).at(link.id)));
              problem.AddResidualBlock(cost_function,
                                       nullptr,
                                       &skp.second.keypoints.at(link.parent_joint).point.position.x,
                                       &skp.second.keypoints.at(link.parent_joint).point.position.y,
                                       &skp.second.keypoints.at(link.parent_joint).point.position.z,
                                       &skp.second.keypoints.at(link.child_joint).point.position.x,
                                       &skp.second.keypoints.at(link.child_joint).point.position.y,
                                       &skp.second.keypoints.at(link.child_joint).point.position.z);
            }
          }
        }
      }

      //      std::cout << ros::Time(m_tracks.src_time) << std::endl;
      //      if (!m_calibrated_links.empty()) {
      //        for (auto& link : m_calibrated_links.begin()->second.begin()->second) {
      //          if (utils::hasKeypoint(m_tracks.skeletons.front().skeleton_parts.begin()->second,
      //          link.second.parent_joint)
      //              && utils::hasKeypoint(m_tracks.skeletons.front().skeleton_parts.begin()->second,
      //                                    link.second.child_joint)) {
      //            if (link.second.id == 2) {
      //              std::cout << "before: l" << link.second.id << " = k" << link.second.parent_joint << " to k"
      //                        << link.second.child_joint << ": " << link.second.length << " output_length: "
      //                        << hiros::skeletons::utils::distance(m_tracks.skeletons.front()
      //                                                               .skeleton_parts.begin()
      //                                                               ->second.keypoints.at(link.second.parent_joint)
      //                                                               .point.position,
      //                                                             m_tracks.skeletons.front()
      //                                                               .skeleton_parts.begin()
      //                                                               ->second.keypoints.at(link.second.child_joint)
      //                                                               .point.position)
      //                        << " length_err: "
      //                        << std::abs(hiros::skeletons::utils::distance(m_tracks.skeletons.front()
      //                                                                        .skeleton_parts.begin()
      //                                                                        ->second.keypoints.at(link.second.parent_joint)
      //                                                                        .point.position,
      //                                                                      m_tracks.skeletons.front()
      //                                                                        .skeleton_parts.begin()
      //                                                                        ->second.keypoints.at(link.second.child_joint)
      //                                                                        .point.position)
      //                                    - link.second.length)
      //                             / link.second.length
      //                        << std::endl;
      //            }
      //          }
      //        }
      //      }

      ceres::Solve(options, &problem, &summary);
      // std::cout << summary.FullReport() << std::endl;

      //      if (!m_calibrated_links.empty()) {
      //        for (auto& link : m_calibrated_links.begin()->second.begin()->second) {
      //          if (utils::hasKeypoint(m_tracks.skeletons.front().skeleton_parts.begin()->second,
      //          link.second.parent_joint)
      //              && utils::hasKeypoint(m_tracks.skeletons.front().skeleton_parts.begin()->second,
      //                                    link.second.child_joint)) {
      //            if (link.second.id == 2) {
      //              std::cout << "after: l" << link.second.id << " = k" << link.second.parent_joint << " to k"
      //                        << link.second.child_joint << ": " << link.second.length << " output_length: "
      //                        << hiros::skeletons::utils::distance(m_tracks.skeletons.front()
      //                                                               .skeleton_parts.begin()
      //                                                               ->second.keypoints.at(link.second.parent_joint)
      //                                                               .point.position,
      //                                                             m_tracks.skeletons.front()
      //                                                               .skeleton_parts.begin()
      //                                                               ->second.keypoints.at(link.second.child_joint)
      //                                                               .point.position)
      //                        << " length_err: "
      //                        << std::abs(hiros::skeletons::utils::distance(m_tracks.skeletons.front()
      //                                                                        .skeleton_parts.begin()
      //                                                                        ->second.keypoints.at(link.second.parent_joint)
      //                                                                        .point.position,
      //                                                                      m_tracks.skeletons.front()
      //                                                                        .skeleton_parts.begin()
      //                                                                        ->second.keypoints.at(link.second.child_joint)
      //                                                                        .point.position)
      //                                    - link.second.length)
      //                             / link.second.length
      //                        << std::endl;
      //            }
      //          }
      //        }
      //      }
    }
  }
}

bool hiros::optimizer::Optimizer::hasCalibration(const int& t_track_id) const
{
  return m_calibrated_links.find(t_track_id) != m_calibrated_links.end();
}
