#include <pluginlib/class_list_macros.h>

#include <vigir_footstep_planner/world_model/terrain_model.h>

namespace vigir_footstep_planning
{
TerrainModel::TerrainModel(const std::string& name, const vigir_generic_params::ParameterSet& params)
  : TerrainModelPlugin(name, params)
{
}

TerrainModel::TerrainModel(const std::string& name)
  : TerrainModelPlugin(name)
{
}

TerrainModel::TerrainModel()
  : TerrainModelPlugin("terrain_model")
{
}

bool TerrainModel::initialize(ros::NodeHandle& nh, const vigir_generic_params::ParameterSet& params)
{
  if (!TerrainModelPlugin::initialize(nh, params))
    return false;

  // get foot dimensions
  getFootSize(nh, foot_size);

  // subscribe
  std::string topic;
  getPluginParam("terrain_model_topic", topic, std::string("/terrain_model"));
  terrain_model_sub = nh.subscribe(topic, 1, &TerrainModel::setTerrainModel, this);

  return true;
}

void TerrainModel::loadParams(const vigir_generic_params::ParameterSet& params)
{
  TerrainModelPlugin::loadParams(params);

  params.getParam("foot_contact_support/min_sampling_steps_x", min_sampling_steps_x);
  params.getParam("foot_contact_support/min_sampling_steps_y", min_sampling_steps_y);
  params.getParam("foot_contact_support/max_sampling_steps_x", max_sampling_steps_x);
  params.getParam("foot_contact_support/max_sampling_steps_y", max_sampling_steps_y);
  params.getParam("foot_contact_support/max_intrusion_z", max_intrusion_z);
  params.getParam("foot_contact_support/max_ground_clearance", max_ground_clearance);
  params.getParam("foot_contact_support/minimal_support", minimal_support);
}

void TerrainModel::reset()
{
  TerrainModelPlugin::reset();

  boost::unique_lock<boost::shared_mutex> lock(terrain_model_shared_mutex);

  if (terrain_model)
    terrain_model->reset();
}

bool TerrainModel::isAccessible(const State& s) const
{
  return s.getGroundContactSupport() >= minimal_support;
}

bool TerrainModel::isAccessible(const State& next, const State& /*current*/) const
{
  return isAccessible(next);
}

bool TerrainModel::isTerrainModelAvailable() const
{
  return terrain_model && terrain_model->hasTerrainModel();
}

void TerrainModel::setTerrainModel(const vigir_terrain_classifier::TerrainModelMsg::ConstPtr& terrain_model)
{
  boost::unique_lock<boost::shared_mutex> lock(terrain_model_shared_mutex);

  // update terrain model
  if (!this->terrain_model)
    this->terrain_model.reset(new vigir_terrain_classifier::TerrainModel(*terrain_model));
  else
    this->terrain_model->fromMsg(*terrain_model);
}

double TerrainModel::getResolution() const
{
  boost::shared_lock<boost::shared_mutex> lock(terrain_model_shared_mutex);
  return terrain_model->getResolution();
}

bool TerrainModel::getPointWithNormal(const pcl::PointNormal& p_search, pcl::PointNormal& p_result) const
{
  boost::shared_lock<boost::shared_mutex> lock(terrain_model_shared_mutex);
  return terrain_model->getPointWithNormal(p_search, p_result);
}

bool TerrainModel::getHeight(double x, double y, double& height) const
{
  boost::shared_lock<boost::shared_mutex> lock(terrain_model_shared_mutex);
  return terrain_model->getHeight(x, y, height);
}

bool TerrainModel::getFootContactSupport(const geometry_msgs::Pose& p, double &support, pcl::PointCloud<pcl::PointXYZI>::Ptr checked_positions) const
{
  tf::Pose p_tf;
  tf::poseMsgToTF(p, p_tf);
  return getFootContactSupport(p_tf, support, checked_positions);
}

bool TerrainModel::getFootContactSupport(const tf::Pose& p, double& support, pcl::PointCloud<pcl::PointXYZI>::Ptr checked_positions) const
{
  if (!getFootContactSupport(p, support, min_sampling_steps_x, min_sampling_steps_y, checked_positions))
    return false;

  // refinement of solution if needed
  if (support == 0.0) // collision, no refinement
  {
    return true;
  }
  else if (support < 0.95)
  {
    if (!getFootContactSupport(p, support, max_sampling_steps_x, max_sampling_steps_y, checked_positions))
      return false;
  }

  return true;
}

bool TerrainModel::getFootContactSupport(const tf::Pose& p, double &support, unsigned int sampling_steps_x, unsigned int sampling_steps_y, pcl::PointCloud<pcl::PointXYZI>::Ptr checked_positions) const
{
  /// TODO: find efficient solution to prevent inconsistency
  //boost::shared_lock<boost::shared_mutex> lock(terrain_model_shared_mutex);

  support = 0.0;

  unsigned int contacts = 0;
  unsigned int unknown = 0;
  unsigned int total = 0;

  tf::Vector3 orig_pos;
  orig_pos.setZ(0.0);

  double foot_size_half_x = 0.5*foot_size.x;
  double foot_size_half_y = 0.5*foot_size.y;

  double sampling_step_x = foot_size.x/(double)(sampling_steps_x-1);
  double sampling_step_y = foot_size.y/(double)(sampling_steps_y-1);

  for (double y = -foot_size_half_y; y <= foot_size_half_y; y+=sampling_step_y)
  {
    orig_pos.setY(y);
    for (double x = -foot_size_half_x; x <= foot_size_half_x; x+=sampling_step_x)
    {
      total++;

      // determine point in world frame and get height at this point
      orig_pos.setX(x);

      const tf::Vector3 &trans_pos = p * orig_pos;

      double height = 0.0;
      if (!getHeight(trans_pos.getX(), trans_pos.getY(), height))
      {
        //ROS_WARN_THROTTLE(1.0, "getFootSupportArea: No height data found at %f/%f", p.getOrigin().getX(), p.getOrigin().getY());
        unknown++;
        continue;
      }

      // diff heights
      double diff = trans_pos.getZ()-height;

      // save evaluated point for visualization
      if (checked_positions)
      {
        pcl::PointXYZI p_checked;
        p_checked.x = trans_pos.getX();
        p_checked.y = trans_pos.getY();
        p_checked.z = trans_pos.getZ();
        p_checked.intensity = std::abs(diff);
        checked_positions->push_back(p_checked);

        //ROS_INFO("%f %f | %f %f | %f", x, y, p.z, height, diff);
      }

      // check diff in z
      if (diff < -max_intrusion_z) // collision -> no support!
        return true;
      else if (diff < max_ground_clearance) // ground contact
        contacts++;
    }
  }

  if (unknown == total)
  {
    return false;
  }
  else
  {
    /// @ TODO: refinement (center of pressure)
    support = static_cast<double>(contacts)/static_cast<double>(total);
    return true;
  }
}

bool TerrainModel::update3DData(geometry_msgs::Pose& p) const
{
  return terrain_model->update3DData(p);
}

bool TerrainModel::update3DData(State& s) const
{
  /// TODO: find efficient solution to prevent inconsistency
  //boost::shared_lock<boost::shared_mutex> lock(terrain_model_shared_mutex);
  bool result = true;

  // get z
  double z = s.getZ();
  if (!getHeight(s.getX(), s.getY(), z))
  {
    //ROS_WARN_THROTTLE(1.0, "No height data found at %f/%f", s.getX(), s.getY());
    result = false;
  }
  else
  {
    s.setZ(z);
  }

  // get roll and pitch
  pcl::PointNormal p_n;
  p_n.x = s.getX();
  p_n.y = s.getY();
  p_n.z = s.getZ();

  if (!getPointWithNormal(p_n, p_n))
  {
    //ROS_WARN_THROTTLE(1.0, "No normal data found at %f/%f", s.getX(), s.getY());
    result = false;
  }
  else
  {
    s.setNormal(p_n.normal_x, p_n.normal_y, p_n.normal_z);
  }

  // determine ground contact support
  double support = 0.0;
  if (!getFootContactSupport(s.getPose(), support))
  {
    //ROS_WARN_THROTTLE(1.0, "Couldn't determine ground contact support at %f/%f", s.getX(), s.getY());
    result = false;
  }
  else
  {
    s.setGroundContactSupport(support);
  }

  return result;
}
}

PLUGINLIB_EXPORT_CLASS(vigir_footstep_planning::TerrainModel, vigir_footstep_planning::TerrainModelPlugin)
