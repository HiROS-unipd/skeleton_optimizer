#ifndef hiros_skeleton_optimizer_Optimizer_h
#define hiros_skeleton_optimizer_Optimizer_h

// ROS dependencies
#include <ros/ros.h>

// Custom Ros Message dependencies
#include "hiros_skeleton_msgs/SkeletonGroup.h"

// Custom External Packages dependencies
#include "skeletons/types.h"

// Internal dependencies
#include "hiros_skeleton_optimizer/Calibrate.h"
#include "hiros_skeleton_optimizer/ChangeId.h"

#define BASH_MSG_RESET "\033[0m"
#define BASH_MSG_GREEN "\033[32m"

namespace hiros {
  namespace optimizer {

    struct LinkInfo
    {
      double confidence;
      double length;
    };

    struct OptimizerParameters
    {
      std::string node_name;

      std::string input_topic_name;
      std::string output_topic_name;

      int number_of_frames_for_calibration;
      double max_calibration_coefficient_of_variation;
      double outlier_threshold;

      bool export_calibration;
      bool load_calibration;
      std::string calibration_file;
    };

    class Optimizer
    {
    public:
      Optimizer() {}
      ~Optimizer() {}

      void configure();
      void start();

    private:
      void stop();

      void parseXml(const XmlRpc::XmlRpcValue& t_xml);
      void checkXmlRpcParam(const std::string& t_tag,
                            const XmlRpc::XmlRpcValue& t_node,
                            const XmlRpc::XmlRpcValue::Type t_type) const;
      bool checkXmlRpcSanity(const std::string& t_tag,
                             const XmlRpc::XmlRpcValue& t_node,
                             const XmlRpc::XmlRpcValue::Type t_type) const;

      void setupRosTopics();

      bool changeId(hiros_skeleton_optimizer::ChangeId::Request& t_req,
                    hiros_skeleton_optimizer::ChangeId::Response& t_res);
      bool calibrate(hiros_skeleton_optimizer::Calibrate::Request& t_req,
                     hiros_skeleton_optimizer::Calibrate::Response& t_res);

      void callback(const hiros_skeleton_msgs::SkeletonGroup& t_msg);

      void changeIds();

      void pushTracksToCalibrationBuffer();
      void calibrate();
      void exportCalibration() const;

      // map<link_id, vector<link_info>>
      std::map<int, std::vector<LinkInfo>>
      computeLinkLengths(const std::vector<hiros::skeletons::types::Skeleton>& t_skeletons) const;
      LinkInfo computeAvg(double& t_coeff_of_variation, const std::vector<LinkInfo>& t_links) const;

      void fixOutliers();
      void fixOutlier(hiros::skeletons::types::Skeleton& t_track, const int& t_link_id, const double& t_link_length);
      void optimize();

      bool hasCalibration(const int& t_track_id) const;

      ros::NodeHandle m_nh{"~"};

      OptimizerParameters m_params{};

      ros::Subscriber m_in_skeleton_group_sub{};
      ros::ServiceServer m_calibrate_srv{};
      ros::ServiceServer m_chande_id_srv{};
      ros::Publisher m_out_msg_pub{};

      skeletons::types::SkeletonGroup m_prev_tracks{};
      skeletons::types::SkeletonGroup m_tracks{};

      std::map<int, int> m_ids_to_change{};

      bool m_acquire_calibration_tracks{false};
      bool m_start_calibration{false};
      std::vector<int> m_ids_to_calibrate{};

      // map<track_id, vector<skeleton>>
      std::map<int, std::vector<hiros::skeletons::types::Skeleton>> m_calibration_buffer{};

      // map<track_id, map<link_id, link_info>>
      std::map<int, std::map<int, LinkInfo>> m_calibrated_links{};

      bool m_configured{false};
    };

  } // namespace optimizer
} // namespace hiros

#endif
