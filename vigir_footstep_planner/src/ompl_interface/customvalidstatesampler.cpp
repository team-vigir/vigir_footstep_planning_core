#include <vigir_footstep_planner/ompl_interface/customvalidstatesampler.h>

namespace vigir_footstep_planning
{
ompl::RNG rng_;

customValidStateSampler::customValidStateSampler(const ompl::base::SpaceInformation *si) : ompl::base::ValidStateSampler(si)
{
  auto footSpaceInit(std::make_shared<ompl::base::SE3StateSpace>()); //foot statespace
  auto selectedFootInit(std::make_shared<ompl::base::DiscreteStateSpace>(0,1)); // selected foot indicator

  ompl::base::RealVectorBounds footSpaceBounds(3); //bounds for foots statespace
  footSpaceBounds.setLow(-10);
  footSpaceBounds.setHigh(10);
  footSpaceBounds.check();
  footSpaceInit->setBounds(footSpaceBounds);

  footSpace = footSpaceInit;
  selectedFoot = selectedFootInit;
  space = footSpace + selectedFoot;

  auto sampleSpace(std::make_shared<ompl::base::SE2StateSpace>()); //sampling 2D statespace

  ompl::base::RealVectorBounds boundsSample(2); //bounds for samplingspace
  boundsSample.setLow(-100);
  boundsSample.setHigh(100);
  sampleSpace->setBounds(boundsSample);

  sampleSpacePtr = sampleSpace;

  lastZVal = 0.0;
}

bool customValidStateSampler::sample(ompl::base::State *state)
{
  double x, y, z, w;

  ompl::base::ScopedState<ompl::base::SE3StateSpace> zeroFoot(footSpace);
  ompl::base::ScopedState<> zeroSelect(selectedFoot);

  zeroFoot->setXYZ(0.0, 0.0, 0.0);
  ompl_helper::radianToQuat(0.0, 0.0, 0.0, &x, &y, &z, &w);
  ompl_helper::setOrientation(zeroFoot->as<ompl::base::SO3StateSpace::StateType>(1), x, y, z, w);

  zeroSelect->as<ompl::base::DiscreteStateSpace::StateType>()->value = 0;

  ompl::base::State* near;
  ompl::base::ScopedState<> converterState(footSpace + selectedFoot);
  zeroFoot >> converterState;
  zeroSelect >> converterState;
  near = (converterState->as<ompl::base::State>());

  return sampleNear(state, near, 5.0);
}

bool customValidStateSampler::sampleNear(ompl::base::State* state, const ompl::base::State* near, const double distance)
{

  ompl::base::ScopedState<> scopedState(space);
  space->copyState(scopedState->as<ompl::base::State>(), near);
  ompl::base::ScopedState<ompl::base::SE3StateSpace> foot(footSpace);
  ompl::base::ScopedState<> select(selectedFoot);

  foot << scopedState;
  select << scopedState;

  ompl::base::ScopedState<ompl::base::SE2StateSpace> sample2D(sampleSpacePtr);
  double distancePerDim = distance / sqrt(2.0);

  //sampling position and rotation in 2D
  sample2D->setXY(rng_.uniformReal(foot.get()->getX() - distancePerDim, foot.get()->getX() + distancePerDim),
                  rng_.uniformReal(foot.get()->getY() - distancePerDim, foot.get()->getY() + distancePerDim));
  sample2D->setYaw(rng_.uniformReal(-M_PI,M_PI));

  ompl::base::ScopedState<ompl::base::SE3StateSpace> sampleFoot(footSpace);
  ompl::base::ScopedState<> sampleSelect(selectedFoot);
  sampleSelect->as<ompl::base::DiscreteStateSpace::StateType>()->value = rng_.uniformInt(0,1);

  if(vigir_footstep_planning::WorldModel::instance().isTerrainModelAvailable())
  {
    State s(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, LEFT);
    double x, y, z, w;

    sampleFoot->setXYZ(sample2D->getX(),sample2D->getY(),lastZVal);

    ompl_helper::radianToQuat(0.0, 0.0, sample2D->getYaw(), &x, &y, &z, &w);
    ompl_helper::setOrientation(sampleFoot->as<ompl::base::SO3StateSpace::StateType>(1), x, y, z, w);
    ompl_helper::getOmplState(&sampleFoot, &s, LEFT);

    vigir_footstep_planning::WorldModel::instance().update3DData(s);

    lastZVal = s.getZ();

    ompl_helper::setOmplState(&sampleFoot, &s);
  }
  else {
    sampleFoot->setXYZ(sample2D->getX(),sample2D->getY(),0.0);
  }

  ompl::base::ScopedState<> converterState(space);
  sampleFoot >> converterState;
  sampleSelect >> converterState;
  space->copyState(state, converterState.get());
  return true;


}
}
