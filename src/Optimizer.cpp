// Standard dependencies
#include <fstream>

// Ceres dependencies
#include "ceres/ceres.h"

// Custom external packages dependencies
#include "skeleton_tracker/utils.h"
#include "skeletons/utils.h"

// Internal dependencies
#include "skeleton_optimizer/CostFunction.h"
#include "skeleton_optimizer/Optimizer.h"

hiros::skeletons::Optimizer::Optimizer() : Node("hiros_skeleton_optimizer") {
  start();
}

hiros::skeletons::Optimizer::~Optimizer() { stop(); }

void hiros::skeletons::Optimizer::start() {
  configure();

  RCLCPP_INFO_STREAM(get_logger(), BASH_MSG_GREEN
                                       << "Hi-ROS Skeleton Optimizer... RUNNING"
                                       << BASH_MSG_RESET);
}

void hiros::skeletons::Optimizer::stop() const {
  RCLCPP_INFO_STREAM(get_logger(), BASH_MSG_GREEN
                                       << "Hi-ROS Skeleton Optimizer... STOPPED"
                                       << BASH_MSG_RESET);

  rclcpp::shutdown();
}

void hiros::skeletons::Optimizer::configure() {
  getParams();
  setupRosTopics();
}

void hiros::skeletons::Optimizer::getParams() {
  getParam("input_topic", params_.input_topic);
  getParam("output_topic", params_.output_topic);
  getParam("number_of_frames_for_calibration",
           params_.number_of_frames_for_calibration);
  getParam("max_calibration_coefficient_of_variation",
           params_.max_calibration_coefficient_of_variation);
  getParam("outlier_threshold", params_.outlier_threshold);
  getParam("export_calibration", params_.export_calibration);
  getParam("load_calibration", params_.load_calibration);
  getParam("calibration_file", params_.calibration_file);

  if (params_.load_calibration) {
    parseCalibConfig();
  }

  if (params_.number_of_frames_for_calibration <= 0) {
    RCLCPP_FATAL_STREAM(
        get_logger(),
        "Hi-ROS Skeleton Optimizer Error: The number of frames to acquire for "
        "the calibration must be greater than 0");
    stop();
    exit(EXIT_FAILURE);
  }

  if (params_.max_calibration_coefficient_of_variation <= 0.0) {
    params_.max_calibration_coefficient_of_variation =
        std::numeric_limits<double>::max();
  }

  if (params_.outlier_threshold <= 0.0) {
    params_.outlier_threshold = std::numeric_limits<double>::max();
  }
}

void hiros::skeletons::Optimizer::parseCalibConfig() {
  std::vector<std::string> tracks{};
  getParam("tracks", tracks);

  for (const auto& track : tracks) {
    int track_id{};
    std::vector<std::string> track_links{};

    getParam(track + ".id", track_id);
    getParam(track + ".links", track_links);

    for (const auto& link : track_links) {
      int link_id{};
      double link_confidence{};
      double link_length{};

      getParam(track + "." + link + ".id", link_id);
      getParam(track + "." + link + ".confidence", link_confidence);
      getParam(track + "." + link + ".length", link_length);

      calibrated_tracks_[track_id][link_id] = {link_confidence, link_length};
    }
  }
}

void hiros::skeletons::Optimizer::setupRosTopics() {
  calibrate_srv_ = create_service<hiros_skeleton_optimizer::srv::Calibrate>(
      "calibrate", std::bind(&Optimizer::calibrateSrv, this,
                             std::placeholders::_1, std::placeholders::_2));
  chande_id_srv_ = create_service<hiros_skeleton_optimizer::srv::ChangeId>(
      "change_id", std::bind(&Optimizer::changeIdSrv, this,
                             std::placeholders::_1, std::placeholders::_2));

  sub_ = create_subscription<hiros_skeleton_msgs::msg::SkeletonGroup>(
      params_.input_topic, 10,
      std::bind(&Optimizer::callback, this, std::placeholders::_1));

  pub_ = create_publisher<hiros_skeleton_msgs::msg::SkeletonGroup>(
      params_.output_topic, 10);
}

void hiros::skeletons::Optimizer::changeIds() {
  for (auto& sk : tracks_.skeletons) {
    if (ids_to_change_.find(sk.id) != ids_to_change_.end()) {
      sk.id = ids_to_change_.at(sk.id);
    }
  }
}

void hiros::skeletons::Optimizer::pushTracksToCalibrationBuffer() {
  for (const auto& sk : tracks_.skeletons) {
    if (std::find(ids_to_calibrate_.begin(), ids_to_calibrate_.end(), sk.id) !=
        ids_to_calibrate_.end()) {
      calibration_buffer_[sk.id].push_back(sk);
    }
  }

  auto n_acquired_frames{
      std::min_element(calibration_buffer_.begin(), calibration_buffer_.end(),
                       [](const auto& s1, const auto& s2) {
                         return s1.second.size() < s2.second.size();
                       })
          ->second.size()};

  if (n_acquired_frames >=
      static_cast<unsigned long>(params_.number_of_frames_for_calibration)) {
    start_calibration_ = true;
  }
}

void hiros::skeletons::Optimizer::calibrate() {
  bool calibration_failed{};
  double coeff_of_variation{};

  for (const auto& track : calibration_buffer_) {
    auto link_lengths{computeLinkLengths(track.second)};

    calibration_failed = false;

    for (const auto& link_length : link_lengths) {
      auto avg_link{computeAvg(coeff_of_variation, link_length.second)};

      if (coeff_of_variation >
          params_.max_calibration_coefficient_of_variation) {
        calibrated_tracks_.erase(track.first);
        calibration_failed = true;
        RCLCPP_WARN_STREAM(
            get_logger(),
            "Hi-ROS Skeleton Optimizer Warning: Calibration of track ID "
                << track.first << " failed. Repeat");
        break;
      }

      calibrated_tracks_[track.first][link_length.first] = avg_link;

      if (calibration_failed) {
        break;
      }
    }
  }

  calibration_buffer_.clear();
  start_calibration_ = false;
  acquire_calibration_tracks_ = false;
}

void hiros::skeletons::Optimizer::exportCalibration() const {
  if (params_.export_calibration) {
    std::ofstream file{};

    file.open(params_.calibration_file, std::ios_base::out);
    file << "/**:" << std::endl;
    file << "  ros__parameters:" << std::endl;
    file << "    tracks: [";
    for (auto idx{0u}; idx < calibrated_tracks_.size(); ++idx) {
      file << "\"t" << std::setfill('0') << std::setw(2) << idx << "\", ";
    }
    file << "]" << std::endl;

    auto track_idx{-1};
    for (const auto& track : calibrated_tracks_) {
      file << "    t" << std::setfill('0') << std::setw(2) << ++track_idx << ":"
           << std::endl;
      file << "      id: " << track.first << std::endl;
      file << "      links: [";
      for (auto idx{0u}; idx < track.second.size(); ++idx) {
        file << "\"l" << std::setfill('0') << std::setw(2) << idx << "\", ";
      }
      file << "]" << std::endl;

      auto link_idx{-1};
      for (const auto& link : track.second) {
        file << "      l" << std::setfill('0') << std::setw(2) << ++link_idx
             << ":" << std::endl;
        file << "        id: " << link.first << std::endl;
        file << "        confidence: " << link.second.confidence << std::endl;
        file << "        length: " << link.second.length << std::endl;
      }
    }

    file.close();
  }
}

std::map<int, std::vector<hiros::skeletons::Optimizer::LinkInfo>>
hiros::skeletons::Optimizer::computeLinkLengths(
    const std::vector<hiros::skeletons::types::Skeleton>& skeletons) const {
  std::map<int, std::vector<hiros::skeletons::Optimizer::LinkInfo>>
      link_lengths{};

  for (const auto& skeleton : skeletons) {
    for (const auto& link : skeleton.links) {
      auto length{utils::linkLength(skeleton, link.id)};

      if (!std::isnan(length)) {
        link_lengths[link.id].push_back({link.confidence, length});
      }
    }
  }

  return link_lengths;
}

hiros::skeletons::Optimizer::LinkInfo hiros::skeletons::Optimizer::computeAvg(
    double& coeff_of_variation,
    const std::vector<hiros::skeletons::Optimizer::LinkInfo>& links) const {
  auto length_sum{std::accumulate(
      links.begin(), links.end(), 0.0, [](double sum, const auto& link) {
        return sum + (link.confidence * link.length);
      })};

  auto length_square_sum{std::accumulate(
      links.begin(), links.end(), 0.0, [](double sum, const auto& link) {
        return sum + (link.confidence * std::pow(link.length, 2));
      })};

  auto confidence_sum{std::accumulate(
      links.begin(), links.end(), 0.0,
      [](double sum, const auto& link) { return sum + link.confidence; })};

  auto mean_length{length_sum / confidence_sum};
  auto stdev_length{std::sqrt(length_square_sum / confidence_sum -
                              mean_length * mean_length)};
  auto mean_conf{confidence_sum / links.size()};

  coeff_of_variation = stdev_length / mean_length;
  LinkInfo res{mean_conf, mean_length};
  return res;
}

void hiros::skeletons::Optimizer::removeOutliers() {
  for (auto& track : tracks_.skeletons) {
    if (calibrated_tracks_.count(track.id) > 0) {
      for (const auto& link_pair : calibrated_tracks_.at(track.id)) {
        if (track.hasLink(link_pair.first)) {
          auto& link{track.getLink(link_pair.first)};
          auto parent_marker_id{link.parent_marker};
          auto child_marker_id{link.child_marker};

          if (track.hasMarker(parent_marker_id) &&
              track.hasMarker(child_marker_id)) {
            removeOutlier(track, link_pair.first, link_pair.second.length);
          }
        }
      }
    }
  }
}

void hiros::skeletons::Optimizer::removeOutlier(
    hiros::skeletons::types::Skeleton& track, const int& link_id,
    const double& link_length) {
  if (!track.hasLink(link_id)) {
    return;
  }

  auto& link{track.getLink(link_id)};

  auto length_error{std::abs(utils::linkLength(track, link_id) - link_length) /
                    link_length};

  if (length_error > params_.outlier_threshold) {
    track.removeMarker(link.child_marker);
  }
}

void hiros::skeletons::Optimizer::optimize() {
  for (const auto& track : tracks_.skeletons) {
    if (hasCalibration(track.id)) {
      ceres::Problem problem{};
      ceres::Solver::Options options{};
      options.logging_type = ceres::LoggingType::SILENT;
      ceres::Solver::Summary summary{};

      for (const auto& link : track.links) {
        if (track.hasMarker(link.parent_marker) &&
            track.hasMarker(link.child_marker)) {
          auto& parent_marker{track.getMarker(link.parent_marker)};
          auto& child_marker{track.getMarker(link.child_marker)};

          auto cost_function{
              new ceres::AutoDiffCostFunction<CostFunction, 1, 1, 1, 1, 1, 1,
                                              1>(new CostFunction(
                  params_.outlier_threshold,
                  calibrated_tracks_.at(track.id).at(link.id).length, track,
                  link))};

          problem.AddResidualBlock(
              cost_function, nullptr,
              const_cast<double*>(&parent_marker.center.pose.position.x()),
              const_cast<double*>(&parent_marker.center.pose.position.y()),
              const_cast<double*>(&parent_marker.center.pose.position.z()),
              const_cast<double*>(&child_marker.center.pose.position.x()),
              const_cast<double*>(&child_marker.center.pose.position.y()),
              const_cast<double*>(&child_marker.center.pose.position.z()));
        }
      }

      ceres::Solve(options, &problem, &summary);
      alignLinkOrientations();
    }
  }
}

bool hiros::skeletons::Optimizer::hasCalibration(const int& track_id) const {
  return calibrated_tracks_.find(track_id) != calibrated_tracks_.end();
}

void hiros::skeletons::Optimizer::alignLinkOrientations() {
  for (auto& track : tracks_.skeletons) {
    for (const auto& link : track.links) {
      utils::alignLinkOrientation(track, link.id);
    }
  }
}

void hiros::skeletons::Optimizer::changeIdSrv(
    const std::shared_ptr<hiros_skeleton_optimizer::srv::ChangeId::Request> req,
    std::shared_ptr<hiros_skeleton_optimizer::srv::ChangeId::Response> res) {
  if (req->to_id > req->from_id) {
    RCLCPP_WARN_STREAM(
        get_logger(),
        "Hi-ROS Skeleton Optimizer Warning: Track IDs can only be lowered. "
        "Cannot change");
    res->ok = false;
    return;
  }

  if (std::find_if(tracks_.skeletons.begin(), tracks_.skeletons.end(),
                   [&](const auto& sk) { return sk.id == req->from_id; }) ==
      tracks_.skeletons.end()) {
    RCLCPP_WARN_STREAM(get_logger(),
                       "Hi-ROS Skeleton Optimizer Warning: Track ID "
                           << req->from_id << " does not exist. Cannot change");
    res->ok = false;
    return;
  }

  if (std::find_if(tracks_.skeletons.begin(), tracks_.skeletons.end(),
                   [&](const auto& sk) { return sk.id == req->to_id; }) !=
      tracks_.skeletons.end()) {
    RCLCPP_WARN_STREAM(get_logger(),
                       "Hi-ROS Skeleton Optimizer Warning: Track ID "
                           << req->to_id << " already exists. Cannot change");
    res->ok = false;
    return;
  }

  RCLCPP_WARN_STREAM(get_logger(),
                     "Hi-ROS Skeleton Optimizer... Change ID from "
                         << req->from_id << " to " << req->to_id);

  auto it{
      std::find_if(ids_to_change_.begin(), ids_to_change_.end(),
                   [&](const auto& p) { return p.second == req->from_id; })};

  // If track with ID from_id had already been changed
  if (it != ids_to_change_.end()) {
    it->second = req->to_id;
  }
  // If track with ID from_id is being changed for the first time
  else {
    ids_to_change_[req->from_id] = req->to_id;
  }

  res->ok = true;
}

void hiros::skeletons::Optimizer::calibrateSrv(
    const std::shared_ptr<hiros_skeleton_optimizer::srv::Calibrate::Request>
        req,
    std::shared_ptr<hiros_skeleton_optimizer::srv::Calibrate::Response> res) {
  if (acquire_calibration_tracks_) {
    RCLCPP_WARN_STREAM(get_logger(),
                       "Hi-ROS Skeleton Optimizer Warning: Cannot start a "
                       "calibration before finishing the previous one");
    res->ok = false;
    return;
  }

  ids_to_calibrate_.clear();

  for (const auto& id : req->track_ids) {
    if (std::find_if(tracks_.skeletons.begin(), tracks_.skeletons.end(),
                     [&](const auto& sk) { return sk.id == id; }) ==
        tracks_.skeletons.end()) {
      RCLCPP_WARN_STREAM(get_logger(),
                         "Hi-ROS Skeleton Optimizer Warning: Track ID "
                             << id << " does not exist. Cannot calibrate");
    } else {
      ids_to_calibrate_.push_back(id);
    }
  }

  if (ids_to_calibrate_.empty()) {
    res->ok = false;
    return;
  }

  std::string tracks_to_calibrate_str{};
  for (const auto& id : ids_to_calibrate_) {
    tracks_to_calibrate_str += " " + std::to_string(id);
  }
  RCLCPP_INFO_STREAM(
      get_logger(), "Hi-ROS Skeleton Optimizer... Starting to calibrate tracks:"
                        << tracks_to_calibrate_str);

  acquire_calibration_tracks_ = true;
  res->ok = true;
}

void hiros::skeletons::Optimizer::callback(
    const hiros_skeleton_msgs::msg::SkeletonGroup& msg) {
  if (!rclcpp::ok()) {
    stop();
    exit(EXIT_FAILURE);
  }

  tracks_ = utils::toStruct(msg);

  changeIds();

  if (acquire_calibration_tracks_) {
    pushTracksToCalibrationBuffer();

    if (start_calibration_) {
      calibrate();
      exportCalibration();
    }
  }

  removeOutliers();
  optimize();
  pub_->publish(utils::toMsg(tracks_));
}
