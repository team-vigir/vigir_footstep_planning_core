#ifndef VALIDSTATESAMPLER_H
#define VALIDSTATESAMPLER_H

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
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
#include <memory.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;
/// @cond IGNORE

class customValidStateSampler : public ompl::base::ValidStateSampler
{
public:
  customValidStateSampler(const ob::SpaceInformation *si);
  /// \brief Destructor
  ~customValidStateSampler() override = default;
  bool sample(ob::State *state) override;
  bool sampleNear(ob::State*, const ob::State*, const double) override;
protected:
  ompl::RNG rng_;
};


#endif // VALIDSTATESAMPLER_H
