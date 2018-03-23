#include <vigir_footstep_planner/customvalidstatesampler.h>


ompl::RNG rng_;
customValidStateSampler::customValidStateSampler(const ompl::base::SpaceInformation *si) : ompl::base::ValidStateSampler(si)
{
  name_ = "my sampler";
}
// Generate a sample in the valid part of the R^3 state space
// Valid states satisfy the following constraints:
// -1<= x,y,z <=1
// if .25 <= z <= .5, then |x|>.8 and |y|>.8
bool customValidStateSampler::sample(ob::State *state)
{
//  auto footLeftSpace(std::make_shared<ompl::base::SE3StateSpace>());
//  auto footRightSpace(std::make_shared<ompl::base::SE3StateSpace>());

//  ompl::base::RealVectorBounds bounds(3);
//  bounds.setLow(-100);
//  bounds.setHigh(100);
//  bounds.check();
//  footLeftSpace->setBounds(bounds);
//  footRightSpace->setBounds(bounds);

//  ompl::base::StateSpacePtr space = footLeftSpace + footRightSpace;
//  ompl::base::ScopedState<> current(space);
//  ompl::base::ScopedState<> next(space);
//  current = state;
//  ompl::base::ScopedState<ompl::base::SE3StateSpace> currentFootLeft(space->as<ompl::base::SE3StateSpace>()->getSubspace(0));
//  ompl::base::ScopedState<ompl::base::SE3StateSpace> currentFootRight(space->as<ompl::base::SE3StateSpace>()->getSubspace(1));
//  current >> currentFootLeft;
//  current >> currentFootRight;
////  double max_distance = 10.0;
////  double left_distance = currentFootLeft.distance(nextFootLeft);
////  double right_distance = currentFootRight.distance(nextFootRight);
////  ROS_INFO("left_distance = %f", left_distance);
////  ROS_INFO("right_distance = %f", right_distance);

//  current.print(std::cout);

  double* val = static_cast<ob::RealVectorStateSpace::StateType*>(state)->values;

  switch(rng_.uniformInt(0,1))
  {
  case 0://move left foot
    break;
  case 1://move right foot
    break;
  }







//  double* val = static_cast<ob::RealVectorStateSpace::StateType*>(state)->values;
//  double z = rng_.uniformReal(-1,1);
//  if (z>.25 && z<.5)
//  {
//    double x = rng_.uniformReal(0,1.8), y = rng_.uniformReal(0,.2);
//    switch(rng_.uniformInt(0,3))
//    {
//    case 0: val[0]=x-1;  val[1]=y-1;
//    case 1: val[0]=x-.8; val[1]=y+.8;
//    case 2: val[0]=y-1;  val[1]=x-1;
//    case 3: val[0]=y+.8; val[1]=x-.8;
//    }
//  }
//  else
//  {
//    val[0] = rng_.uniformReal(-1,1);
//    val[1] = rng_.uniformReal(-1,1);
//  }
//  val[2] = z;
////  assert(si_->isValid(state));
//  return true;
}
// We don't need this in the example below.
bool customValidStateSampler::sampleNear(ob::State*, const ob::State*, const double)
{
  throw ompl::Exception("MyValidStateSampler::sampleNear", "not implemented");
  return false;
}

