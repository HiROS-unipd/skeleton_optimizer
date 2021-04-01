#ifndef hiros_skeleton_optimizer_Optimizer_h
#define hiros_skeleton_optimizer_Optimizer_h

// ROS dependencies
#include <ros/ros.h>

// Custom Ros Message dependencies
#include "hiros_skeleton_msgs/MarkerSkeletonGroup.h"

// Custom External Packages dependencies
#include "skeletons/types.h"

// Internal dependencies
#include "hiros_skeleton_optimizer/Calibrate.h"
#include "hiros_skeleton_optimizer/ChangeId.h"
#include "skeleton_optimizer/utils.h"

#define BASH_MSG_RESET "\033[0m"
#define BASH_MSG_GREEN "\033[32m"

namespace hiros {
  namespace optimizer {

    struct OptimizerParameters
    {
      std::string node_name;

      std::string input_topic_name;
      std::string output_topic_name;

      bool define_links;
      int number_of_frames_for_calibration;
      double max_calibration_coefficient_of_variation;
      double outlier_threshold;
    };

    class Optimizer
    {
    public:
      Optimizer();
      ~Optimizer();

      void configure();
      void start();

    private:
      void stop();
      void setupRosTopics();
      std::string extractImageQualityFromTopicName(const std::string& t_topic_name) const;

      bool changeId(hiros_skeleton_optimizer::ChangeId::Request& t_req,
                    hiros_skeleton_optimizer::ChangeId::Response& t_res);
      bool calibrate(hiros_skeleton_optimizer::Calibrate::Request& t_req,
                     hiros_skeleton_optimizer::Calibrate::Response& t_res);

      void callback(hiros_skeleton_msgs::MarkerSkeletonGroupConstPtr t_skeleton_group_msg);

      void changeIds();

      void pushTracksToCalibrationBuffer();
      void calibrate();
      void computeLinkLenghts(const std::vector<hiros::skeletons::types::MarkerSkeleton>& t_skeletons);
      hiros::optimizer::utils::Link computeLinkLength(const hiros::optimizer::utils::Link& t_link,
                                                      const int& t_marker_group_id,
                                                      const hiros::skeletons::types::MarkerSkeleton& t_skeleton) const;

      void fixOutliers();
      void fixOutlier(const hiros::optimizer::utils::Link& t_link,
                      const int& t_marker_group_id,
                      const hiros::skeletons::types::MarkerSkeleton& t_track);
      void optimize();

      bool hasCalibration(const int& t_track_id) const;

      ros::NodeHandle m_nh;
      std::string m_node_namespace;

      OptimizerParameters m_params;

      ros::Subscriber m_in_skeleton_group_sub;
      ros::ServiceServer m_calibrate_srv;
      ros::ServiceServer m_chande_id_srv;
      ros::Publisher m_out_msg_pub;

      skeletons::types::MarkerSkeletonGroup m_prev_tracks;
      skeletons::types::MarkerSkeletonGroup m_tracks;

      std::map<int, int> m_ids_to_change;

      bool m_acquire_calibration_tracks;
      bool m_start_calibration;
      std::vector<int> m_ids_to_calibrate;

      // map<track_id, vector<marker_skeleton>>
      std::map<int, std::vector<hiros::skeletons::types::MarkerSkeleton>> m_calibration_buffer;

      // map<marker_group_id, vector<link>>
      std::map<int, std::vector<utils::Link>> m_links;

      // map<track_id, map<marker_group_id, map<link_id, link>>>
      std::map<int, std::map<int, std::map<int, utils::Link>>> m_calibrated_links;

      // map<track_id, map<marker_group_id, map<link_id, vector<link>>>>
      std::map<int, std::map<int, std::map<int, std::vector<utils::Link>>>> m_link_lengths_vector;

      bool m_configured;
    };

  } // namespace optimizer
} // namespace hiros

#endif
