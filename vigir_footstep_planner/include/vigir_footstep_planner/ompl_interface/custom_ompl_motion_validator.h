#ifndef CUSTOM_OMPL_MOTION_VALIDATOR
#define CUSTOM_OMPL_MOTION_VALIDATOR

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
class customOmplMotionValidator : public ompl::base::MotionValidator
{
public:
  /// \brief Constructor
  customOmplMotionValidator(ompl::base::SpaceInformation* si);
  /// \brief Constructor
  customOmplMotionValidator(const ompl::base::SpaceInformationPtr &si);
  /// \brief Destructor
  ~customOmplMotionValidator() override = default;
  /// \brief Checks Motion from s1 to s2
  bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const override;
  bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2, std::pair<ompl::base::State*, double> &lastValid) const override;
};
}


#endif // CUSTOM_OMPL_MOTION_VALIDATOR
