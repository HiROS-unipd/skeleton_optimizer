// Internal dependencies
#include "skeleton_optimizer/CostFunction.h"

hiros::optimizer::CostFunction::CostFunction(const double& t_max_length_error,
                                             const hiros::skeletons::types::Position& t_joint_0,
                                             const hiros::skeletons::types::Position& t_joint_1,
                                             const hiros::optimizer::utils::Link& t_link)
  : m_max_length_error(t_max_length_error)
  , m_joint_0(t_joint_0)
  , m_joint_1(t_joint_1)
  , m_link(t_link)
{}

hiros::optimizer::CostFunction::~CostFunction() {}
