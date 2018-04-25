#ifndef CUSTOMSTATEVALIDITYCHECKER_H
#define CUSTOMSTATEVALIDITYCHECKER_H

#include <vigir_footstep_planner/footstep_planner.h>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/PlannerStatus.h>
#include <ompl/config.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/MotionValidator.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/base/ProblemDefinition.h>

namespace vigir_footstep_planning
{
class CustomStateValidityChecker : public ompl::base::StateValidityChecker
{
private:
  ompl::base::StateSpacePtr space;
public:
  /// \brief Constructor
  CustomStateValidityChecker(const ompl::base::SpaceInformationPtr &si);
  /// \brief checks if State is valid
  bool isValid(const ompl::base::State *state) const;

};
}

#endif // CUSTOMSTATEVALIDITYCHECKER_H
