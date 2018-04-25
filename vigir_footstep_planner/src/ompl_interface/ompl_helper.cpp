#include <vigir_footstep_planner/ompl_interface/ompl_helper.h>
namespace vigir_footstep_planning
{
ompl_helper::ompl_helper()
{

}
void ompl_helper::radianToQuat(double roll, double pitch, double yaw, double* x, double* y, double* z, double* w)
{
  double pitchHalf = pitch / 2.0;
  double yawHalf = yaw / 2.0;
  double rollHalf = roll / 2.0;

  double sinp = sin(pitchHalf);
  double siny = sin(yawHalf);
  double sinr = sin(rollHalf);
  double cosp = cos(pitchHalf);
  double cosy = cos(yawHalf);
  double cosr = cos(rollHalf);

  *x = sinr * cosp * cosy - cosr * sinp * siny;
  *y = ( cosr * sinp * cosy + sinr * cosp * siny );
  *z = ( cosr * cosp * siny - sinr * sinp * cosy );
  *w = ( cosr * cosp * cosy + sinr * sinp * siny );
}

void ompl_helper::quatToRadian(ompl_base::SO3StateSpace::StateType* rotation, double* roll, double* pitch, double* yaw)
{
  //roll
  double sinr = +2.0 * (rotation->w * rotation->x + rotation->y * rotation->z);
  double cosr = +1.0 - 2.0 * (rotation->x * rotation->x + rotation->y * rotation->y);
  *roll = atan2(sinr, cosr);

  //pitch
  double sinp = +2.0 * (rotation->w * rotation->y - rotation->z * rotation->x);
  if (fabs(sinp) >= 1)
    *pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  else
    *pitch = asin(sinp);

  //yaw
  double siny = +2.0 * (rotation->w * rotation->z + rotation->x * rotation->y);
  double cosy = +1.0 - 2.0 * (rotation->y * rotation->y + rotation->z * rotation->z);
  *yaw = atan2(siny, cosy);
}
void ompl_helper::setOrientation(ompl_base::SO3StateSpace::StateType* rotation, double x, double y, double z, double w)
{
  rotation->x = x;
  rotation->y = y;
  rotation->z = z;
  rotation->w = w;
}
void ompl_helper::setOmplState(ompl_base::ScopedState<ompl_base::SE3StateSpace>* foot, State* s)
{
  foot->get()->setXYZ(s->getX(),s->getY(),s->getZ());

  double x,y,z,w;

  ompl_helper::radianToQuat(s->getRoll(), s->getPitch(), s->getYaw(), &x, &y, &z, &w);
  ompl_helper::setOrientation(foot->get()->as<ompl_base::SO3StateSpace::StateType>(1), x, y, z, w);

}
void ompl_helper::getOmplState(ompl_base::ScopedState<ompl_base::SE3StateSpace>* foot, State* s, Leg leg)
{
  double x,y,z;

  x = foot->get()->getX();
  y = foot->get()->getY();
  z = foot->get()->getZ();

  ompl_base::SO3StateSpace::StateType *rot = foot->get()->as<ompl::base::SO3StateSpace::StateType>(1);

  double roll, pitch, yaw;

  ompl_helper::quatToRadian(rot, &roll, &pitch, &yaw);

  s->setRPY(roll,pitch,yaw);
  s->setLeg(leg);
  s->setX(x);
  s->setY(y);
  s->setZ(z);
}
}
