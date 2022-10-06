// Internal dependencies
#include "skeleton_optimizer/CostFunction.h"

hiros::skeletons::CostFunction::CostFunction(
    const double& max_length_error, const double& link_length,
    const hiros::skeletons::types::Skeleton& skeleton,
    const hiros::skeletons::types::Link& link)
    : max_length_error_(max_length_error),
      link_length_(link_length),
      link_(link) {
  if (skeleton.hasMarker(link.parent_marker) &&
      skeleton.hasMarker(link.child_marker)) {
    parent_marker_ = skeleton.getMarker(link.parent_marker);
    child_marker_ = skeleton.getMarker(link.child_marker);
  }
}

hiros::skeletons::CostFunction::~CostFunction() {}

tf2::Vector3 hiros::skeletons::CostFunction::getQuatAxis(
    tf2::Vector3 axis, const tf2::Quaternion& orientation) const {
  axis.normalize();

  auto closest_cartesian_axis{closestCartesianAxis(
      tf2::quatRotate(orientation.inverse(), axis).normalized())};

  // Axis of the link orientation SoR to be aligned to the link axis
  return tf2::quatRotate(orientation, closest_cartesian_axis).normalized();
}

tf2::Vector3 hiros::skeletons::CostFunction::closestCartesianAxis(
    const tf2::Vector3& vec) const {
  auto closest_axis_idx{vec.closestAxis()};  // 0: x, 1: y, 2: z

  tf2::Vector3DoubleData signs{};
  signs.m_floats[0] = vec.x() >= 0 ? 1 : -1;
  signs.m_floats[1] = vec.y() >= 0 ? 1 : -1;
  signs.m_floats[2] = vec.z() >= 0 ? 1 : -1;

  tf2::Vector3DoubleData closest_axis_serialized{0, 0, 0};
  closest_axis_serialized.m_floats[closest_axis_idx] =
      signs.m_floats[closest_axis_idx];

  tf2::Vector3 closest_axis{};
  closest_axis.deSerialize(closest_axis_serialized);

  return closest_axis;
}
