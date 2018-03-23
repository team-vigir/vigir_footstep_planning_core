#include <vigir_footstep_planner/custom_ompl_motion_validator.h>

namespace vigir_footstep_planning
{

customOmplMotionValidator::customOmplMotionValidator(ompl::base::SpaceInformation* si) : ompl::base::MotionValidator(si)
{
}

customOmplMotionValidator::customOmplMotionValidator(const ompl::base::SpaceInformationPtr &si) : ompl::base::MotionValidator(si)
{
}
bool customOmplMotionValidator::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const
{
  auto footLeftSpace(std::make_shared<ompl::base::SE3StateSpace>());
  auto footRightSpace(std::make_shared<ompl::base::SE3StateSpace>());

  ompl::base::RealVectorBounds bounds(3);
  bounds.setLow(-100);
  bounds.setHigh(100);
  bounds.check();
  footLeftSpace->setBounds(bounds);
  footRightSpace->setBounds(bounds);

  ompl::base::StateSpacePtr space = footLeftSpace + footRightSpace;
  ompl::base::ScopedState<> current(space);
  ompl::base::ScopedState<> next(space);
  current = s1;
  next = s2;
  ompl::base::ScopedState<ompl::base::SE3StateSpace> currentFootLeft(space->as<ompl::base::SE3StateSpace>()->getSubspace(0));
  ompl::base::ScopedState<ompl::base::SE3StateSpace> currentFootRight(space->as<ompl::base::SE3StateSpace>()->getSubspace(1));
  current >> currentFootLeft;
  current >> currentFootRight;
  ompl::base::ScopedState<ompl::base::SE3StateSpace> nextFootLeft(space->as<ompl::base::SE3StateSpace>()->getSubspace(0));
  ompl::base::ScopedState<ompl::base::SE3StateSpace> nextFootRight(space->as<ompl::base::SE3StateSpace>()->getSubspace(1));
  next >> nextFootLeft;
  next >> nextFootRight;
  double max_distance = 10.0;
  double left_distance = currentFootLeft.distance(nextFootLeft);
  double right_distance = currentFootRight.distance(nextFootRight);
  ROS_INFO("left_distance = %f", left_distance);
  ROS_INFO("right_distance = %f", right_distance);
  if((left_distance <= max_distance && right_distance <= max_distance) || (right_distance <= max_distance && left_distance <= max_distance))
  {
    return true;
  }
  else
  {
    return false;
  }

}
bool customOmplMotionValidator::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2, std::pair<ompl::base::State*, double> &lastValid) const
{

  return true;

}
}
