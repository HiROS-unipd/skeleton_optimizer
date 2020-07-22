// Standard dependencies
#include <algorithm> // TODO: remove
#include <cmath>
#include <numeric>

// Internal dependencies
#include "skeleton_optimizer/utils.h"

hiros::optimizer::utils::Link hiros::optimizer::utils::computeAvgLink(double& t_coeff_of_variation,
                                                                      const std::vector<Link>& t_links)
{
  double length_sum = std::accumulate(t_links.begin(), t_links.end(), 0.0, [](double sum, const Link& link) {
    return sum + (link.confidence * link.length);
  });

  double sq_length_sum = std::accumulate(t_links.begin(), t_links.end(), 0.0, [](double sq_sum, const Link& link) {
    return sq_sum + (link.confidence * std::pow(link.length, 2));
  });

  double conf_sum = std::accumulate(
    t_links.begin(), t_links.end(), 0.0, [](double conf_sum, const Link& link) { return conf_sum + link.confidence; });

  double mean_length = length_sum / conf_sum;
  double stdev_length = std::sqrt(sq_length_sum / conf_sum - mean_length * mean_length);
  double mean_conf = conf_sum / t_links.size();

  t_coeff_of_variation = stdev_length / mean_length;

  Link res = t_links.front();
  res.confidence = mean_conf;
  res.length = mean_length;

  return res;
}
