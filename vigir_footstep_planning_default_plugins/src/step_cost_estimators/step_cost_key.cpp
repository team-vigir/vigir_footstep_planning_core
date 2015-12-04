#include <vigir_footstep_planning_default_plugins/step_cost_estimators/step_cost_key.h>

namespace vigir_footstep_planning
{
StepCostKey::StepCostKey(const std::vector<double> &state, double cell_size, double angle_bin_size)
  : cell_size(cell_size)
  , angle_bin_size(angle_bin_size)
{
  setState(state);
}

StepCostKey::StepCostKey(const State &left_foot, const State &right_foot, const State &swing_foot, double cell_size, double angle_bin_size)
  : cell_size(cell_size)
  , angle_bin_size(angle_bin_size)
{
  std::vector<double> state;
  transformStates(left_foot, right_foot, swing_foot, state);
  setState(state);
}

StepCostKey::StepCostKey(const StepCostKey &other)
  : std::vector<int>(other)
  , cell_size(other.cell_size)
  , angle_bin_size(other.angle_bin_size)
{
}

bool StepCostKey::operator== (const StepCostKey &other) const
{
  unsigned long int size = std::min(this->size(), other.size());

  for (unsigned long int i = 0; i < size; i++)
    {
      if ((*this)[i] == other[i])
        continue;

      return false;
    }

  return true;
}

bool StepCostKey::operator< (const StepCostKey &other) const
{
  unsigned long int size = std::min(this->size(), other.size());

  for (unsigned long int i = 0; i < size; i++)
    {
      if ((*this)[i] == other[i])
        continue;

      if ((*this)[i] < other[i])
        return true;

      if ((*this)[i] > other[i])
        return false;
    }

  return false;
}

void StepCostKey::setState(const std::vector<double> &state)
{
  this->clear();
  this->reserve(state.size());

  unsigned int i = 0;

  this->push_back(cont_2_disc(state[i++], cell_size));
  this->push_back(cont_2_disc(state[i++], cell_size));
  //this->push_back(cont_2_disc(state[i++], cell_size));
  //this->push_back(cont_2_disc(state[i++], angle_bin_size));
  //this->push_back(cont_2_disc(state[i++], angle_bin_size));
  this->push_back(cont_2_disc(state[i++], angle_bin_size));

  this->push_back(cont_2_disc(state[i++], cell_size));
  this->push_back(cont_2_disc(state[i++], cell_size));
  //this->push_back(cont_2_disc(state[i++], cell_size));
  //this->push_back(cont_2_disc(state[i++], angle_bin_size));
  //this->push_back(cont_2_disc(state[i++], angle_bin_size));
  this->push_back(cont_2_disc(state[i++], angle_bin_size));

  //this->push_back(state[i++]);

  //ROS_INFO("%f %f %f | %f %f %f", state[0], state[1], state[2], state[3], state[4], state[5]);
  //ROS_INFO("%i %i %i | %i %i %i", (*this)[0], (*this)[1], (*this)[2], (*this)[3], (*this)[4], (*this)[5]);
  //ROS_INFO("--------");
}

void StepCostKey::getState(std::vector<double> &state) const
{
  state.clear();
  state.reserve(this->size());

  unsigned int i = 0;

  state.push_back(disc_2_cont((*this)[i++], cell_size));
  state.push_back(disc_2_cont((*this)[i++], cell_size));
  //state.push_back(disc_2_cont((*this)[i++], cell_size));
  //state.push_back(disc_2_cont((*this)[i++], angle_bin_size));
  //state.push_back(disc_2_cont((*this)[i++], angle_bin_size));
  state.push_back(disc_2_cont((*this)[i++], angle_bin_size));

  state.push_back(disc_2_cont((*this)[i++], cell_size));
  state.push_back(disc_2_cont((*this)[i++], cell_size));
  //state.push_back(disc_2_cont((*this)[i++], cell_size));
  //state.push_back(disc_2_cont((*this)[i++], angle_bin_size));
  //state.push_back(disc_2_cont((*this)[i++], angle_bin_size));
  state.push_back(disc_2_cont((*this)[i++], angle_bin_size));

  //state.push_back((*this)[i++]);
}

void StepCostKey::transformStates(const State &left_foot, const State &right_foot, const State &swing_foot, std::vector<double> &state) const
{
  /*
     * State definition:
     * A state is given by the left foot poses (before and after swinging).
     * Both poses must be given in the frame of the right standing foot.
     */

  tf::Pose left_foot_pose = left_foot.getPose();
  tf::Pose right_foot_pose = right_foot.getPose();
  tf::Pose swing_foot_pose = swing_foot.getPose();

  // transform left foot pose after executing the step relative to right foot
  tf::Pose left_foot_before;
  tf::Pose left_foot_after;

  // swing foot is left foot -> trivial
  if (swing_foot.getLeg() == LEFT)
  {
    // determine transformation, so right standing foot is origin
    tf::Transform world_to_right_foot = tf::Transform(right_foot_pose.getRotation(), right_foot_pose.getOrigin());
    world_to_right_foot = world_to_right_foot.inverse();

    // transform left standing foot pose into right foot frame
    left_foot_before = world_to_right_foot * left_foot_pose;

    // transform left swing pose into right foot frame
    left_foot_after = world_to_right_foot * swing_foot_pose;
  }
  // mirror all states to swinging left foot
  else // data_point.swing_foot.foot_index == RIGHT
  {
    // determine transformation, so left standing foot is origin
    tf::Transform world_to_left_foot = tf::Transform(left_foot_pose.getRotation(), left_foot_pose.getOrigin());
    world_to_left_foot = world_to_left_foot.inverse();

    // transform right standing foot pose into left foot frame
    tf::Pose right_foot_before;
    right_foot_before = world_to_left_foot * right_foot_pose;

    // transform right swing pose into left foot frame
    tf::Pose right_foot_after;
    right_foot_after = world_to_left_foot * swing_foot_pose;

    // now we are mirroring all data to right foot frame
    mirrorPoseOnXPlane(left_foot_before, right_foot_before);
    mirrorPoseOnXPlane(left_foot_after, right_foot_after);
  }

  state.clear();
  state.reserve(STATE_DIM);
  double r, p, y;

  // left foot before swing
  left_foot_before.getBasis().getRPY(r, p, y);
  state.push_back(left_foot_before.getOrigin().getX());
  state.push_back(left_foot_before.getOrigin().getY());
  //state.push_back(left_foot_before.getOrigin().getZ());
  //state.push_back(r);
  //state.push_back(p);
  state.push_back(y);

  // left foot after swing
  left_foot_after.getBasis().getRPY(r, p, y);
  state.push_back(left_foot_after.getOrigin().getX());
  state.push_back(left_foot_after.getOrigin().getY());
  //state.push_back(left_foot_after.getOrigin().getZ());
  //state.push_back(r);
  //state.push_back(p);
  state.push_back(y);

  // terrain type
  //state.push_back(data_point.terrain_type;

  //ROS_INFO("------- foot --------------------------");
  //ROS_INFO("%f %f %f / %f %f %f", state[0], state[1], state[2], state[3], state[4], state[5]);
  //ROS_INFO("%f %f %f / %f %f %f", state[6], state[7], state[8], state[9], state[10], state[11]);
}

void StepCostKey::mirrorPoseOnXPlane(tf::Pose &mirror, const tf::Pose &orig) const
{
  mirror.setBasis(orig.getBasis());
  tf::Matrix3x3& basis = mirror.getBasis();
  basis[0][1] = -basis[0][1];
  basis[1][0] = -basis[1][0];
  basis[2][1] = -basis[2][1];

  mirror.setOrigin(orig.getOrigin());
  tf::Vector3& origin = mirror.getOrigin();
  origin[1] = -origin[1];


//  double r, p, y;
//  tf::Matrix3x3(orig.getRotation()).getRPY(r, p, y);
//  mirror.setRotation(tf::createQuaternionFromRPY(-r, p, -y));

//  tf::Vector3 v = orig.getOrigin();
//  v.setY(-v.getY());
//  mirror.setOrigin(v);


//  tfScalar m[16];
//  ROS_INFO("------------------");
//  mirror.getOpenGLMatrix(m);
//  ROS_INFO("%f %f %f %f", m[0], m[4], m[8], m[12]);
//  ROS_INFO("%f %f %f %f", m[1], m[5], m[9], m[13]);
//  ROS_INFO("%f %f %f %f", m[2], m[6], m[10], m[14]);
//  ROS_INFO("%f %f %f %f", m[3], m[7], m[11], m[15]);

//  ROS_INFO("-");
//  orig.getOpenGLMatrix(m);

//  ROS_INFO("%f %f %f %f", m[0], m[4], m[8], m[12]);
//  ROS_INFO("%f %f %f %f", m[1], m[5], m[9], m[13]);
//  ROS_INFO("%f %f %f %f", m[2], m[6], m[10], m[14]);
//  ROS_INFO("%f %f %f %f", m[3], m[7], m[11], m[15]);

//  ROS_INFO("-");

//  ROS_INFO("%f %f %f %f", m[0], -m[4], m[8], m[12]);
//  ROS_INFO("%f %f %f %f", -m[1], m[5], m[9], -m[13]);
//  ROS_INFO("%f %f %f %f", m[2], -m[6], m[10], m[14]);
//  ROS_INFO("%f %f %f %f", m[3], m[7], m[11], m[15]);
}
}

std::size_t hash_value(const vigir_footstep_planning::StepCostKey &key)
{
  std::size_t seed = 0;
  for (unsigned int i = 0; i < STATE_DIM; i++)
  {
    boost::hash_combine(seed, key[i]);
  }
  return seed;
}
