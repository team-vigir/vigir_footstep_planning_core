#ifndef PROJECTION_FOR_SPACE_H
#define PROJECTION_FOR_SPACE_H

//#include<vigir_footstep_planner/footstep_planner.h>

//#include <ompl/base/SpaceInformation.h>
//#include <ompl/base/StateSpace.h>
//#include <ompl/base/spaces/SE3StateSpace.h>
//#include <ompl/base/spaces/SE2StateSpace.h>
//#include <ompl/base/spaces/RealVectorStateSpace.h>
//#include <ompl/base/spaces/RealVectorBounds.h>
//#include <ompl/geometric/SimpleSetup.h>
//#include <ompl/base/ScopedState.h>
//#include <ompl/base/PlannerStatus.h>
//#include <ompl/config.h>
//#include <ompl/base/StateValidityChecker.h>
//#include <ompl/base/MotionValidator.h>
//#include <ompl/geometric/planners/rrt/RRTConnect.h>
//#include <ompl/base/ProblemDefinition.h>
//#include <ompl/base/Path.h>
//#include <ompl/base/spaces/DiscreteStateSpace.h>
//#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>
//#include <ompl/base/samplers/informed/PathLengthDirectInfSampler.h>
//#include <ompl/base/samplers/InformedStateSampler.h>
//#include <ompl/base/samplers/UniformValidStateSampler.h>
//#include <ompl/base/ProjectionEvaluator.h>

namespace vigir_footstep_planning
{
class ProjectionForSpace : public ompl::base::ProjectionEvaluator
{
public:
ProjectionForSpace(const ompl::base::StateSpacePtr &space) : ompl::base::ProjectionEvaluator(space)
{
}
virtual unsigned int getDimension(void) const
{
    return 2;
}
virtual void defaultCellSizes(void)
{
    cellSizes_.resize(2);
    cellSizes_[0] = 0.1;
    cellSizes_[1] = 0.1;
}
virtual void project(const ompl::base::State *state, ompl::base::EuclideanProjection &projection) const
{
    const double *values = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
    projection(0) = values[0];
    projection(1) = values[1];
}
};

#endif // PROJECTION_FOR_SPACE_H
}
