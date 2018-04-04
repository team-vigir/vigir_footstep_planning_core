#include <vigir_footstep_planner/customstatevaliditychecker.h>
namespace vigir_footstep_planning
{

CustomStateValidityChecker::CustomStateValidityChecker(const ompl::base::SpaceInformationPtr &si) : ompl::base::StateValidityChecker(si)
{
  space = si->getStateSpace();
}
bool CustomStateValidityChecker::isValid(const ompl::base::State *state) const
{
  ompl::base::ScopedState<> scopedState(space);
  space->copyState(scopedState->as<ompl::base::State>(), state);
  ompl::base::ScopedState<ompl::base::SE3StateSpace> foot(space->as<ompl::base::SE3StateSpace>()->getSubspace(0));
  ompl::base::ScopedState<> selectFoot(space->as<ompl::base::SE3StateSpace>()->getSubspace(1));
  foot << scopedState;
  selectFoot << scopedState;
  if(vigir_footstep_planning::WorldModel::instance().isTerrainModelAvailable())
  {

  }
  else
  {
    return ((foot.get()->getZ()) == 0);
  }
}

}
