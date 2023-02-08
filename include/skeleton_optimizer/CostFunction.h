#ifndef hiros_skeleton_optimizer_CostFunction_h
#define hiros_skeleton_optimizer_CostFunction_h

// Custom external packages dependencies
#include "ceres/jet.h"
#include "skeletons/types.h"
#include "skeletons/utils.h"

namespace hiros {
namespace skeletons {

class CostFunction {
 public:
  CostFunction(const double& link_length,
               const hiros::skeletons::types::Skeleton& skeleton,
               const hiros::skeletons::types::Link& link)
      : link_length_(link_length), link_(link) {
    if (skeleton.hasMarker(link.parent_marker) &&
        skeleton.hasMarker(link.child_marker)) {
      parent_marker_ = skeleton.getMarker(link.parent_marker);
      child_marker_ = skeleton.getMarker(link.child_marker);
    }
  }

  ~CostFunction() {}

  template <typename T>
  bool operator()(const T* const x_p, const T* const x_c, T* e) const {
    if (skeletons::utils::isNaN(parent_marker_.center.pose.position) ||
        skeletons::utils::isNaN(child_marker_.center.pose.position)) {
      return false;
    }

    auto length_error{abs(
        hiros::skeletons::utils::distance(parent_marker_.center.pose.position,
                                          child_marker_.center.pose.position) -
        link_length_)};
    auto length_weight{
        std::min(std::max(0.001, length_error / link_length_), 0.999)};

    e[0] = length_weight * lengthError(x_p, x_c);
    e[1] = length_weight * 0.1 * positionError(x_p, x_c);
    e[2] = length_weight * 0.1 * directionError(x_p, x_c);

    return true;
  }

 private:
  template <typename T>
  T lengthError(const T* const x_p, const T* const x_c) const {
    // Squared length of the optimized link
    auto sq_link_length{ceres::pow(x_p[0] - x_c[0], 2) +
                        ceres::pow(x_p[1] - x_c[1], 2) +
                        ceres::pow(x_p[2] - x_c[2], 2)};

    // If sqrt() is not differentiable -> x ~ 0
    if (sq_link_length == 0.) {
      return T(0.) - link_length_;
    }

    // If sqrt() is differentiable
    return ceres::sqrt(sq_link_length) - link_length_;
  }

  template <typename T>
  T positionError(const T* const x_p, const T* x_c) const {
    // Squared distance between optimized and original parent marker
    auto sq_xp_err{
        ceres::pow(x_p[0] - parent_marker_.center.pose.position.x(), 2) +
        ceres::pow(x_p[1] - parent_marker_.center.pose.position.y(), 2) +
        ceres::pow(x_p[2] - parent_marker_.center.pose.position.z(), 2)};
    // Squared distance between optimized and original child marker
    auto sq_xc_err{
        ceres::pow(x_c[0] - child_marker_.center.pose.position.x(), 2) +
        ceres::pow(x_c[1] - child_marker_.center.pose.position.y(), 2) +
        ceres::pow(x_c[2] - child_marker_.center.pose.position.z(), 2)};

    // Initialize errors to 0 (used if sqrt() is not differentiable -> x ~ 0)
    T xp_err(0.);
    T xc_err(0.);

    // If sqrt() is differentiable
    if (sq_xp_err > 0.) {
      xp_err = ceres::sqrt(sq_xp_err);
    }
    if (sq_xc_err > 0.) {
      xc_err = ceres::sqrt(sq_xc_err);
    }

    // Return sum of parent and child marker errors
    return xp_err + xc_err;
  }

  template <typename T>
  T directionError(const T* const x_p, const T* const x_c) const {
    // Normalized vector before optimization
    auto orig_v{(child_marker_.center.pose.position -
                 parent_marker_.center.pose.position)
                    .normalize()};

    // Optimized vector
    auto vx{x_c[0] - x_p[0]};
    auto vy{x_c[1] - x_p[1]};
    auto vz{x_c[2] - x_p[2]};

    // Squared norm of the optimized vector
    auto den{ceres::pow(vx, 2) + ceres::pow(vy, 2) + ceres::pow(vz, 2)};

    if (den == 0.) {
      // Cannot normalize the vector
      return T(0.);
    }

    // Norm of the optimized vector
    den = ceres::sqrt(den);

    // Normalized optimized vector
    auto vnx{vx / den};
    auto vny{vy / den};
    auto vnz{vz / den};

    // Difference between original and optimized vectors
    auto diff_x{vnx - orig_v.x()};
    auto diff_y{vny - orig_v.y()};
    auto diff_z{vnz - orig_v.z()};

    // Squared magnitude of the difference vector -> squared distance
    auto sq_dist{ceres::pow(diff_x, 2) + ceres::pow(diff_y, 2) +
                 ceres::pow(diff_z, 2)};

    // If sqrt() is not differentiable -> x ~ 0
    if (sq_dist == 0.) {
      return T(0.);
    }

    // If sqrt() is differentiable
    return ceres::sqrt(sq_dist);
  }

  double link_length_{};
  hiros::skeletons::types::Link link_{};
  hiros::skeletons::types::Marker parent_marker_{};
  hiros::skeletons::types::Marker child_marker_{};
};
}  // namespace skeletons
}  // namespace hiros

#endif
