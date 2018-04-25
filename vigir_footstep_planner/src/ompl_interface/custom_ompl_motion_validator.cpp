#include <vigir_footstep_planner/ompl_interface/custom_ompl_motion_validator.h>

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

  auto footSpaceInit(std::make_shared<ompl::base::SE3StateSpace>()); //statespace for one foot
  auto selectedFootInit(std::make_shared<ompl::base::DiscreteStateSpace>(0,1)); //statespace determening wich foot

  ompl::base::RealVectorBounds footSpaceBounds(3); //bounds of foot statespace
  footSpaceBounds.setLow(-10);
  footSpaceBounds.setHigh(10);
  footSpaceBounds.check();
  footSpaceInit->setBounds(footSpaceBounds);

  ompl::base::StateSpacePtr footSpace = footSpaceInit;
  ompl::base::StateSpacePtr selectedFoot = selectedFootInit;
  ompl::base::StateSpacePtr space = footSpace + selectedFoot;

  ompl::base::ScopedState<> scopedState1(space); //whole states
  ompl::base::ScopedState<> scopedState2(space);
  space->copyState(scopedState1->as<ompl::base::State>(), s1); //copy given states
  space->copyState(scopedState2->as<ompl::base::State>(), s2);
  ompl::base::ScopedState<ompl::base::SE3StateSpace> foot1(space->as<ompl::base::SE3StateSpace>()->getSubspace(0)); //feet states
  ompl::base::ScopedState<ompl::base::SE3StateSpace> foot2(space->as<ompl::base::SE3StateSpace>()->getSubspace(0));
  ompl::base::ScopedState<> selectFoot1(space->as<ompl::base::SE3StateSpace>()->getSubspace(1)); //selected foot indicators
  ompl::base::ScopedState<> selectFoot2(space->as<ompl::base::SE3StateSpace>()->getSubspace(1));

  foot1 << scopedState1; //copy feet states
  foot2 << scopedState2;
  selectFoot1 << scopedState1; //copy selected feet
  selectFoot2 << scopedState2;

  State state1(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, LEFT); //convert to different state implementation
  Leg currentLeg = selectFoot1->as<ompl::base::DiscreteStateSpace::StateType>()->value == 0 ? RIGHT : LEFT;
  ompl_helper::getOmplState(&foot1, &state1, currentLeg);
  State state2(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, LEFT);
  currentLeg = selectFoot2->as<ompl::base::DiscreteStateSpace::StateType>()->value == 0 ? RIGHT : LEFT;
  ompl_helper::getOmplState(&foot2, &state2, currentLeg);

  if (RobotModel::instance().isReachable(state1, state2))
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

  return checkMotion(s1, s2);

}
}
