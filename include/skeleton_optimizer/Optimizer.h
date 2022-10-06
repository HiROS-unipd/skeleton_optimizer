#ifndef hiros_skeleton_optimizer_Optimizer_h
#define hiros_skeleton_optimizer_Optimizer_h

// ROS dependencies
#include <rclcpp/rclcpp.hpp>

// Custom ROS message dependencies
#include "hiros_skeleton_msgs/msg/skeleton_group.hpp"

// Custom external packages dependencies
#include "skeletons/types.h"

// Internal dependencies
#include "hiros_skeleton_optimizer/srv/calibrate.hpp"
#include "hiros_skeleton_optimizer/srv/change_id.hpp"

#define BASH_MSG_RESET "\033[0m"
#define BASH_MSG_GREEN "\033[32m"

namespace hiros {
namespace skeletons {

class Optimizer : public rclcpp::Node {
 public:
  Optimizer();
  ~Optimizer();

 private:
  struct LinkInfo {
    double confidence{};
    double length{};
  };

  struct Parameters {
    std::string input_topic{};
    std::string output_topic{};

    int number_of_frames_for_calibration{};
    double max_calibration_coefficient_of_variation{};
    double outlier_threshold{};

    bool export_calibration{false};
    bool load_calibration{false};
    std::string calibration_file{};
  };

  template <typename T>
  bool getParam(const std::string& name, T& parameter) {
    declare_parameter<T>(name);
    return get_parameter(name, parameter);
  }

  void start();
  void stop() const;
  void configure();

  void getParams();
  void parseCalibConfig();
  void setupRosTopics();

  void changeIds();

  void pushTracksToCalibrationBuffer();
  void calibrate();
  void exportCalibration() const;

  // map<link_id, vector<link_info>>
  std::map<int, std::vector<LinkInfo>> computeLinkLengths(
      const std::vector<hiros::skeletons::types::Skeleton>& skeletons) const;
  LinkInfo computeAvg(double& coeff_of_variation,
                      const std::vector<LinkInfo>& links) const;

  void fixOutliers();
  void fixOutlier(hiros::skeletons::types::Skeleton& track, const int& link_id,
                  const double& link_length);
  void optimize();

  bool hasCalibration(const int& track_id) const;
  void alignLinkOrientations();

  void changeIdSrv(
      const std::shared_ptr<hiros_skeleton_optimizer::srv::ChangeId::Request>
          req,
      std::shared_ptr<hiros_skeleton_optimizer::srv::ChangeId::Response> res);
  void calibrateSrv(
      const std::shared_ptr<hiros_skeleton_optimizer::srv::Calibrate::Request>
          req,
      std::shared_ptr<hiros_skeleton_optimizer::srv::Calibrate::Response> res);

  void callback(const hiros_skeleton_msgs::msg::SkeletonGroup& msg);

  rclcpp::Subscription<hiros_skeleton_msgs::msg::SkeletonGroup>::SharedPtr
      sub_{};
  rclcpp::Publisher<hiros_skeleton_msgs::msg::SkeletonGroup>::SharedPtr pub_{};

  rclcpp::Service<hiros_skeleton_optimizer::srv::Calibrate>::SharedPtr
      calibrate_srv_{};
  rclcpp::Service<hiros_skeleton_optimizer::srv::ChangeId>::SharedPtr
      chande_id_srv_{};

  bool acquire_calibration_tracks_{false};
  bool start_calibration_{false};
  std::vector<int> ids_to_calibrate_{};
  std::map<int, int> ids_to_change_{};

  Parameters params_{};

  skeletons::types::SkeletonGroup prev_tracks_{};
  skeletons::types::SkeletonGroup tracks_{};

  // map<track_id, vector<skeleton>>
  std::map<int, std::vector<hiros::skeletons::types::Skeleton>>
      calibration_buffer_{};

  // map<track_id, map<link_id, link_info>>
  std::map<int, std::map<int, LinkInfo>> calibrated_tracks_{};
};

}  // namespace skeletons
}  // namespace hiros

#endif
