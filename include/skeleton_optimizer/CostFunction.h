#ifndef hiros_skeleton_optimizer_CostFunction_h
#define hiros_skeleton_optimizer_CostFunction_h

// Custom External Packages dependencies
#include "skeletons/types.h"
#include "skeletons/utils.h"

namespace hiros {
  namespace optimizer {

    class CostFunction
    {
    public:
      CostFunction(const double& t_max_length_error,
                   const hiros::skeletons::types::Marker& t_marker_0,
                   const hiros::skeletons::types::Marker& t_marker_1,
                   const double& t_link_length);
      ~CostFunction();

      template <typename T>
      bool operator()(const T* const x0,
                      const T* const y0,
                      const T* const z0,
                      const T* const x1,
                      const T* const y1,
                      const T* const z1,
                      T* residual) const
      {
        double length_error =
          abs(hiros::skeletons::utils::distance(m_marker_0.center.pose.position, m_marker_1.center.pose.position)
              - m_link_length)
          / m_link_length;

        double link_length_weight = std::min(std::max(0.01, length_error / m_max_length_error), 0.99);
        double joint_position_weight = 1 - link_length_weight;

        auto joint_position_distance =
          (pow(x0[0] - m_marker_0.center.pose.position.x(), 2) + pow(y0[0] - m_marker_0.center.pose.position.y(), 2)
           + pow(z0[0] - m_marker_0.center.pose.position.z(), 2) + pow(x1[0] - m_marker_1.center.pose.position.x(), 2)
           + pow(y1[0] - m_marker_1.center.pose.position.y(), 2) + pow(z1[0] - m_marker_1.center.pose.position.z(), 2));

        auto link_length_distance =
          abs((pow(x0[0] - x1[0], 2) + pow(y0[0] - y1[0], 2) + pow(z0[0] - z1[0], 2) - pow(m_link_length, 2)));

        residual[0] = sqrt(joint_position_weight * joint_position_distance + link_length_weight * link_length_distance);
        return true;
      }

    private:
      double m_max_length_error;
      hiros::skeletons::types::Marker m_marker_0;
      hiros::skeletons::types::Marker m_marker_1;
      double m_link_length;
    };
  } // namespace optimizer
} // namespace hiros

#endif
