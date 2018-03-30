#ifndef VALIDSTATESAMPLER_H
#define VALIDSTATESAMPLER_H

#include <math.h>

#include <vigir_footstep_planner/footstep_planner.h>
#include <vigir_footstep_planner/ompl_helper.h>

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

namespace ob = ompl::base;
namespace og = ompl::geometric;


namespace vigir_footstep_planning
{
class customValidStateSampler : public ompl::base::ValidStateSampler
{
private:
  ompl::base::StateSpacePtr space, sampleSpacePtr;
  std::shared_ptr<ompl::base::UniformValidStateSampler> uniform2DSampler;
  double lastZVal;
public:

//  /** \brief Construct a sampler that only generates states with a heuristic solution estimate that is less than the cost of the current solution using the default informed sampler for the current optimization objective. Requires a function pointer to a method to query the cost of the current solution. */
//  customValidStateSampler(const ompl::base::ProblemDefinitionPtr &probDefn, unsigned int maxNumberCalls, const GetCurrentCostFunc &costFunc);

//  /** \brief Construct a sampler that only generates states with a heuristic solution estimate that is less than the cost of the current solution using the provided informed sampler. Requires a function pointer to a method to query the cost of the current solution. */
//  customValidStateSampler(const ompl::base::ProblemDefinitionPtr &probDefn, const GetCurrentCostFunc &costFunc, const ompl::base::InformedSamplerPtr &infSampler);

//  /// \brief Destructor
//  ~customValidStateSampler() override = default;

//  /** \brief Sample uniformly in the subset of the state space whose heuristic solution estimates are less than the current best cost (as defined by the pointer passed at construction). By default just calls sampleUniform(State*, Cost) with cost given by the member variable. */
//  void sampleUniform(ompl::base::State *statePtr) override;

//  /** \brief By default sampleUniformNear throws. This can be overloaded by a specific informed sampler if desired. */
//  void sampleUniformNear(State *statePtr, const State *near, const double distance) override;

//  /** \brief By default sampleGaussian throws. This can be overloaded by a specific informed sampler if desired. */
//  void sampleGaussian(State *statePtr, const State *mean, const double stdDev) override;


  customValidStateSampler(const ob::SpaceInformation *si);

  bool sample(ob::State *state) override;
  bool sampleNear(ob::State* state, const ob::State* near, const double distance) override;
protected:
  ompl::RNG rng_;
};
}


#endif // VALIDSTATESAMPLER_H
