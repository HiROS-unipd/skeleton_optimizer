#ifndef hiros_skeleton_optimizer_utils_h
#define hiros_skeleton_optimizer_utils_h

// Standard dependencies
#include <memory> // TODO: remove
#include <string>
#include <vector>

// Custom external dependencies
#include "skeletons/types.h"

namespace hiros {
  namespace optimizer {
    namespace utils {

      struct Link
      {
        int id;
        std::string name;

        int parent_joint;
        int child_joint;

        double confidence;
        double length;
      };

      Link computeAvgLink(double& t_coeff_of_variation, const std::vector<Link>& t_links);

    } // namespace utils

  } // namespace optimizer
} // namespace hiros

#endif
