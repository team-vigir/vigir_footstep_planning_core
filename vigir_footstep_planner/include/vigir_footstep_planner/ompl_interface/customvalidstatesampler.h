#ifndef VALIDSTATESAMPLER_H
#define VALIDSTATESAMPLER_H

#include <math.h>

#include <vigir_footstep_planner/footstep_planner.h>
#include <vigir_footstep_planner/ompl_interface/ompl_helper.h>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/PlannerStatus.h>
#include <ompl/config.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/MotionValidator.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/Path.h>
#include <ompl/base/spaces/DiscreteStateSpace.h>
#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>
#include <ompl/base/samplers/informed/PathLengthDirectInfSampler.h>
#include <ompl/base/samplers/InformedStateSampler.h>
#include <ompl/base/samplers/UniformValidStateSampler.h>
#include <memory.h>

namespace vigir_footstep_planning
{
class customValidStateSampler : public ompl::base::ValidStateSampler
{
private:
  ompl::base::StateSpacePtr space, sampleSpacePtr, footSpace, selectedFoot;
  std::shared_ptr<ompl::base::UniformValidStateSampler> uniform2DSampler;
  double lastZVal; //heigt position of last sampled state
public:
  /// \brief Constructor
  customValidStateSampler(const ompl::base::SpaceInformation *si);
  /// \brief returns new random state in statespace
  bool sample(ompl::base::State *state) override;
  /// \brief returns new random state in max distance of given state "near"
  bool sampleNear(ompl::base::State* state, const ompl::base::State* near, const double distance) override;
protected:
  ompl::RNG rng_;
};
}


#endif // VALIDSTATESAMPLER_H
