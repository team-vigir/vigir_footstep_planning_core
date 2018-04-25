#include <vigir_footstep_planner/ompl_interface/customstatevaliditychecker.h>
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
  ompl::base::ScopedState<ompl::base::SE3StateSpace> foot(space->as<ompl::base::SE3StateSpace>()->getSubspace(0)); //foot statespace
  ompl::base::ScopedState<> selectFoot(space->as<ompl::base::SE3StateSpace>()->getSubspace(1)); //selected foot indicator
  foot << scopedState;
  selectFoot << scopedState;
  if(vigir_footstep_planning::WorldModel::instance().isTerrainModelAvailable())
  {
    double x = foot.get()->getX();
    double y = foot.get()->getY();
    double *height;
    vigir_footstep_planning::WorldModel::instance().getHeight(x,y,*height);
    if ((foot.get()->getZ()) == *height)
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  else
  {
    if ((foot.get()->getZ()) == 0.0)
    {
      return true;
    }
    else
    {
      return false;
    }
  }
}

}
