// Internal dependencies
#include "skeleton_optimizer/CostFunction.h"

hiros::optimizer::CostFunction::CostFunction(const double& t_max_length_error,
                                             const double& t_link_length,
                                             const hiros::skeletons::types::Skeleton& t_skeleton,
                                             const hiros::skeletons::types::Link& t_link)
  : m_max_length_error(t_max_length_error)
  , m_link_length(t_link_length)
  , m_link(t_link)
{
  if (t_skeleton.hasMarker(t_link.parent_marker) && t_skeleton.hasMarker(t_link.child_marker)) {
    m_parent_marker = t_skeleton.getMarker(t_link.parent_marker);
    m_child_marker = t_skeleton.getMarker(t_link.child_marker);
  }
}

hiros::optimizer::CostFunction::~CostFunction() {}

tf2::Vector3 hiros::optimizer::CostFunction::getQuatAxis(tf2::Vector3 t_axis,
                                                         const tf2::Quaternion& t_orientation) const
{
  t_axis.normalize();

  auto closest_cartesian_axis = closestCartesianAxis(tf2::quatRotate(t_orientation.inverse(), t_axis).normalized());

  // Axis of the link orientation SoR to be aligned to the link axis
  return tf2::quatRotate(t_orientation, closest_cartesian_axis).normalized();
}

tf2::Vector3 hiros::optimizer::CostFunction::closestCartesianAxis(const tf2::Vector3& t_vec) const
{
  auto closest_axis_idx = t_vec.closestAxis(); // 0: x, 1: y, 2: z

  tf2::Vector3DoubleData signs;
  signs.m_floats[0] = t_vec.x() >= 0 ? 1 : -1;
  signs.m_floats[1] = t_vec.y() >= 0 ? 1 : -1;
  signs.m_floats[2] = t_vec.z() >= 0 ? 1 : -1;

  tf2::Vector3DoubleData closest_axis_serialized{0, 0, 0};
  closest_axis_serialized.m_floats[closest_axis_idx] = signs.m_floats[closest_axis_idx];

  tf2::Vector3 closest_axis;
  closest_axis.deSerialize(closest_axis_serialized);

  return closest_axis;
}
