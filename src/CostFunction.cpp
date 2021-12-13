// Internal dependencies
#include "skeleton_optimizer/CostFunction.h"

hiros::optimizer::CostFunction::CostFunction(const double& t_max_length_error,
                                             const hiros::skeletons::types::Marker& t_marker_0,
                                             const hiros::skeletons::types::Marker& t_marker_1,
                                             const double& t_link_length)
  : m_max_length_error(t_max_length_error)
  , m_marker_0(t_marker_0)
  , m_marker_1(t_marker_1)
  , m_link_length(t_link_length)
{}

hiros::optimizer::CostFunction::~CostFunction() {}
