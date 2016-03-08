#include <vigir_footstep_planning_default_plugins/step_cost_estimators/map_step_cost_estimator.h>

#include <pluginlib/class_list_macros.h>



namespace vigir_footstep_planning
{
MapStepCostEstimator::MapStepCostEstimator()
  : StepCostEstimatorPlugin("map_step_cost_estimator")
{
}

bool MapStepCostEstimator::loadParams(const vigir_generic_params::ParameterSet& params)
{
  if (!StepCostEstimatorPlugin::loadParams(params))
    return false;

  std::string filename;
  params.getParam("map_step_cost_estimator/file", filename);

  loadFromFile(filename);

  return true;
}

bool MapStepCostEstimator::getCost(const State& left_foot, const State& right_foot, const State& swing_foot, double& cost, double& cost_multiplier, double& risk, double& risk_multiplier) const
{
  cost = 0.0;
  cost_multiplier = 1.0;
  risk = 0.0;
  risk_multiplier = 1.0;

  const State& stand_foot = swing_foot.getLeg() == LEFT ? right_foot : left_foot;
  double foot_dist = euclidean_distance(stand_foot.getX(), stand_foot.getY(), swing_foot.getX(), swing_foot.getY());

  const State& swing_foot_before = swing_foot.getLeg() == LEFT ? left_foot : right_foot;
  double dist = 0.5*euclidean_distance(swing_foot_before.getX(), swing_foot_before.getY(), swing_foot.getX(), swing_foot.getY());

  if (dist > 0.3 || foot_dist > 0.45)
  {
    risk = 1.0;
  }
  else
  {
    StepCostKey key(left_foot, right_foot, swing_foot, cell_size, angle_bin_size);
    boost::unordered_map<StepCostKey, std::pair<double, double> >::const_iterator iter = cost_map.find(key);

    if (iter == cost_map.end())
    {
      risk = 1.0;
    }
    else
    {
      risk = iter->second.second <= 0.25 ? abs(iter->second.first) : 1.0;
    }
  }

  //std::vector<double> state;
  //key.getState(state);
  //ROS_INFO("%f %f %f | %f %f %f", swing_foot_before.getX(), swing_foot_before.getY(), swing_foot_before.getYaw(), swing_foot.getX(), swing_foot.getY(), swing_foot.getYaw());
  //ROS_INFO("%f %f %f | %f %f %f", state[0], state[1], state[2], state[3], state[4], state[5]);
  //ROS_INFO("------");
  //if (risk_cost < 1.0 && state[5] > 0.0)
  //  ROS_INFO("Cost: %f + %f + %f Yaw: %f", dist, risk_cost, default_step_cost, state[5]);

  cost = dist + 2*risk*risk;
  return true;
}

void MapStepCostEstimator::insert(const std::vector<double> &key, const std::pair<double, double> &entry)
{
  cost_map[StepCostKey(key, cell_size, angle_bin_size)] = entry;
}

void MapStepCostEstimator::loadFromFile(const std::string &filename)
{
  // open file
  std::ifstream file;
  file.open(filename.c_str(), std::ios::in | std::ios::binary);

  if (!file.is_open())
  {
    ROS_ERROR("Could not read from file %s", filename.c_str());
    return;
  }

  /* read model */

  // boundaries
  read<float>(file, min_x);
  read<float>(file, max_x);
  read<float>(file, min_y);
  read<float>(file, max_y);
  read<float>(file, min_yaw);
  read<float>(file, max_yaw);

  // resolution
  float temp;
  read<float>(file, temp);
  cell_size = temp;
  read<uint32_t>(file, num_angle_bin);
  angle_bin_size = 2.0*M_PI/(double)num_angle_bin;

  // size
  uint32_t dim_size, num_elements;
  read<uint32_t>(file, num_elements);
  read<uint32_t>(file, dim_size);
  dim_size -= 2;

  // load map
  std::vector<double> key;
  key.resize(dim_size);
  std::pair<double, double> entry;

  cost_map.clear();
  for (unsigned int i = 0; i < num_elements; i++)
  {
    // load key
    for (unsigned int d = 0; d < dim_size; d++)
    {
      float k;
      read<float>(file, k);
      key[d] = k;
    }

    // load entry
    float c, s;
    read<float>(file, c); // cost estimation
    read<float>(file, s); // variance
    entry.first = c;
    entry.second = s;

    // insert to map
    insert(key, entry);
  }

  ROS_INFO("Boundaries (x/y/yaw): %f %f %f %f %f %f", min_x, max_x, min_y, max_y, min_yaw, max_yaw);
  ROS_INFO("Resolution: long: %f rot: %u %f", cell_size, num_angle_bin, angle_bin_size);
  ROS_INFO("Loaded step cost map: %u dimensions and %lu entries", dim_size, cost_map.size());

  // close file
  file.close();
}
}

PLUGINLIB_EXPORT_CLASS(vigir_footstep_planning::MapStepCostEstimator, vigir_footstep_planning::StepCostEstimatorPlugin)
