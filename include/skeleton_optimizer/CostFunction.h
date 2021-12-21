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
                   const double& t_link_length,
                   const hiros::skeletons::types::Skeleton& t_skeleton,
                   const hiros::skeletons::types::Link& t_link);
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
        if (skeletons::utils::isNaN(m_parent_marker.center.pose.position)
            || skeletons::utils::isNaN(m_child_marker.center.pose.position)) {
          return false;
        }

        double length_error = abs(
          hiros::skeletons::utils::distance(m_parent_marker.center.pose.position, m_child_marker.center.pose.position)
          - m_link_length);

        double length_weight = std::min(std::max(0.001, length_error / m_max_length_error), 0.999);
        double position_weight = 0.5 * (1 - length_weight);
        double orientation_weight = 1 - length_weight - position_weight;

        auto length_distance = lengthDistance(x0, y0, z0, x1, y1, z1);
        auto position_distance = positionDistance(x0, y0, z0, x1, y1, z1);

        residual[0] = sqrt(length_weight * length_distance + position_weight * position_distance);

        if (!skeletons::utils::isNaN(m_link.center.pose.orientation)) {
          auto orientation_distance = orientationDistance(x0, y0, z0, x1, y1, z1);
          if (orientation_distance > 0.) {
            residual[0] = sqrt(position_weight * position_distance + length_weight * length_distance
                               + orientation_weight * orientation_distance);
          }
        }

        return true;
      }

    private:
      template <typename T>
      T lengthDistance(const T* const x0,
                       const T* const y0,
                       const T* const z0,
                       const T* const x1,
                       const T* const y1,
                       const T* const z1) const
      {
        return abs((pow(x0[0] - x1[0], 2) + pow(y0[0] - y1[0], 2) + pow(z0[0] - z1[0], 2) - pow(m_link_length, 2)));
      }

      template <typename T>
      T positionDistance(const T* const x0,
                         const T* const y0,
                         const T* const z0,
                         const T* const x1,
                         const T* const y1,
                         const T* const z1) const
      {
        return pow(x0[0] - m_parent_marker.center.pose.position.x(), 2)
               + pow(y0[0] - m_parent_marker.center.pose.position.y(), 2)
               + pow(z0[0] - m_parent_marker.center.pose.position.z(), 2)
               + pow(x1[0] - m_child_marker.center.pose.position.x(), 2)
               + pow(y1[0] - m_child_marker.center.pose.position.y(), 2)
               + pow(z1[0] - m_child_marker.center.pose.position.z(), 2);
      }

      template <typename T>
      T orientationDistance(const T* const x0,
                            const T* const y0,
                            const T* const z0,
                            const T* const x1,
                            const T* const y1,
                            const T* const z1) const
      {
        // Axis of the link orientation SoR to be aligned to the link axis
        auto quat_axis = getQuatAxis(m_parent_marker.center.pose.position - m_child_marker.center.pose.position,
                                     m_link.center.pose.orientation);

        // x, y, z components of the link axis computed from parent and child markers positions
        auto dx = x0[0] - x1[0];
        auto dy = y0[0] - y1[0];
        auto dz = z0[0] - z1[0];
        auto den = sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2));
        dx = dx / den;
        dy = dy / den;
        dz = dz / den;

        // Rotation angle to align the link orientation to the link axis
        auto rot_angle = acos(fmin(fmax(T(-1.), quat_axis.x() * dx + quat_axis.y() * dy + quat_axis.z() * dz), T(1.)));

        return pow(rot_angle, 2);
      }

      tf2::Vector3 getQuatAxis(tf2::Vector3 t_axis, const tf2::Quaternion& t_orientation) const;
      tf2::Vector3 closestCartesianAxis(const tf2::Vector3& t_vec) const;

      double m_max_length_error;
      double m_link_length;
      hiros::skeletons::types::Link m_link;
      hiros::skeletons::types::Marker m_parent_marker;
      hiros::skeletons::types::Marker m_child_marker;
    };
  } // namespace optimizer
} // namespace hiros

#endif
