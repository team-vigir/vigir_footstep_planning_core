#ifndef PROJECTION_FOR_SPACE_H
#define PROJECTION_FOR_SPACE_H

namespace vigir_footstep_planning
{
class ProjectionForSpace : public ompl::base::ProjectionEvaluator
{
public:
  /// \brief Constructor
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
