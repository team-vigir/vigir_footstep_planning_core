#include <vigir_footstep_planner/custom_ompl_motion_validator.h>

namespace vigir_footstep_planning
{

customOmplMotionValidator::customOmplMotionValidator(ompl::base::SpaceInformation* si) : ompl::base::MotionValidator(si)
{
//  space = si->getStateSpace();
}

customOmplMotionValidator::customOmplMotionValidator(const ompl::base::SpaceInformationPtr &si) : ompl::base::MotionValidator(si)
{
}
bool customOmplMotionValidator::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const
{
//  auto footLeftSpace(std::make_shared<ompl::base::SE3StateSpace>());
//  auto footRightSpace(std::make_shared<ompl::base::SE3StateSpace>());

//  ompl::base::RealVectorBounds bounds(3);
//  bounds.setLow(-100);
//  bounds.setHigh(100);
//  bounds.check();
//  footLeftSpace->setBounds(bounds);
//  footRightSpace->setBounds(bounds);

//  ompl::base::StateSpacePtr space = footLeftSpace + footRightSpace;
//  ompl::base::ScopedState<> current(space);
//  ompl::base::ScopedState<> next(space);
//  current = s1;
//  next = s2;
//  ompl::base::ScopedState<ompl::base::SE3StateSpace> currentFootLeft(space->as<ompl::base::SE3StateSpace>()->getSubspace(0));
//  ompl::base::ScopedState<ompl::base::SE3StateSpace> currentFootRight(space->as<ompl::base::SE3StateSpace>()->getSubspace(1));
//  current >> currentFootLeft;
//  current >> currentFootRight;
//  ompl::base::ScopedState<ompl::base::SE3StateSpace> nextFootLeft(space->as<ompl::base::SE3StateSpace>()->getSubspace(0));
//  ompl::base::ScopedState<ompl::base::SE3StateSpace> nextFootRight(space->as<ompl::base::SE3StateSpace>()->getSubspace(1));
//  next >> nextFootLeft;
//  next >> nextFootRight;
//  double max_distance = 5.0;
//  double left_distance = currentFootLeft.distance(nextFootLeft);
//  double right_distance = currentFootRight.distance(nextFootRight);
////  ROS_INFO("left_distance = %f", left_distance);
////  ROS_INFO("right_distance = %f", right_distance);
//  if((left_distance <= 0.3 && right_distance <= max_distance) || (right_distance <= 0.3 && left_distance <= max_distance))
//  {
//    return true;
//  }
//  else
//  {
//    return false;
//  }

  auto footSpaceInit(std::make_shared<ompl_base::SE3StateSpace>());
  auto selectedFootInit(std::make_shared<ompl::base::DiscreteStateSpace>(0,1));
  ompl_base::RealVectorBounds footSpaceBounds(3);
  footSpaceBounds.setLow(-10);
  footSpaceBounds.setHigh(10);
  footSpaceBounds.check();
  footSpaceInit->setBounds(footSpaceBounds);
  ompl_base::StateSpacePtr footSpace = footSpaceInit;
  ompl_base::StateSpacePtr selectedFoot = selectedFootInit;
  ompl_base::StateSpacePtr space = footSpace + selectedFoot;
//  space = newSpace;
  ompl::base::ScopedState<> scopedState1(space);
  ompl::base::ScopedState<> scopedState2(space);
  space->copyState(scopedState1->as<ompl::base::State>(), s1);
  space->copyState(scopedState2->as<ompl::base::State>(), s2);
  ompl::base::ScopedState<ompl::base::SE3StateSpace> foot1(space->as<ompl::base::SE3StateSpace>()->getSubspace(0));
  ompl::base::ScopedState<ompl::base::SE3StateSpace> foot2(space->as<ompl::base::SE3StateSpace>()->getSubspace(0));
  ompl::base::ScopedState<> selectFoot1(space->as<ompl::base::SE3StateSpace>()->getSubspace(1));
  ompl::base::ScopedState<> selectFoot2(space->as<ompl::base::SE3StateSpace>()->getSubspace(1));
  foot1 << scopedState1;
  foot2 << scopedState2;
  selectFoot1 << scopedState1;
  selectFoot2 << scopedState2;
  State state1(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, LEFT);
  Leg currentLeg = selectFoot1->as<ompl::base::DiscreteStateSpace::StateType>()->value == 0 ? RIGHT : LEFT;
  ompl_helper::getOmplState(&foot1, &state1, currentLeg);
  State state2(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, LEFT);
  currentLeg = selectFoot2->as<ompl::base::DiscreteStateSpace::StateType>()->value == 0 ? RIGHT : LEFT;
  ompl_helper::getOmplState(&foot2, &state2, currentLeg);
//  space->printState(scopedState1.get(), std::cout);
//  space->printState(scopedState2.get(), std::cout);
  if (RobotModel::instance().isReachable(state1, state2))
  {
//    ROS_INFO("motion valid");
    return true;
  }
  else
  {
//    ROS_INFO("motion invalid");
    return false;
  }


}
bool customOmplMotionValidator::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2, std::pair<ompl::base::State*, double> &lastValid) const
{

  return true;

}
}
