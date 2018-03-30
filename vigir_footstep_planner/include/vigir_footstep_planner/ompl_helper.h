#ifndef OMPL_HELPER_H
#define OMPL_HELPER_H

#include<vigir_footstep_planner/footstep_planner.h>

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

class ompl_helper
{
public:
  ompl_helper();

  /// @brief turns given roll, pitch and yaw into quaternions.
  static void radianToQuat(double roll, double pitch, double yaw, double* x, double* y, double* z, double* w);
  /// @brief turns given quaternions into roll, pitch and yaw.
  static void quatToRadian(ompl::base::SO3StateSpace::StateType* rotation, double* roll, double* pitch, double* yaw);
  /// @brief sets the orientation of given SO3 OMPL state
  static void setOrientation(ompl::base::SO3StateSpace::StateType* rotation, double x, double y, double z, double w);
  /// @brief converts State to OMPL state
  static void setOmplState(ompl::base::ScopedState<ompl::base::SE3StateSpace> *foot, State* s);
  /// @brief converts OMPL state to State
  static void getOmplState(ompl::base::ScopedState<ompl::base::SE3StateSpace> *foot, State* s, Leg leg);
};
}

#endif // OMPL_HELPER_H
