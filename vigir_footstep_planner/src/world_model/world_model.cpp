#include <vigir_footstep_planner/world_model/world_model.h>

namespace vigir_footstep_planning
{
WorldModel::Ptr WorldModel::singelton = WorldModel::Ptr();

WorldModel::WorldModel()
{
}

WorldModel::Ptr& WorldModel::Instance()
{
  if (!singelton)
    singelton.reset(new WorldModel());
  return singelton;
}

void WorldModel::loadPlugins()
{
  // get collision check plugins
  PluginManager::getPluginsByType(Instance()->collision_check_models);

  ROS_INFO("[WorldModel] Plugins loaded:");
  for (std::vector<CollisionCheckPlugin::Ptr>::const_iterator itr = Instance()->collision_check_models.begin(); itr != Instance()->collision_check_models.end(); itr++)
  {
    const CollisionCheckPlugin::Ptr& collision_check_model = *itr;
    if (collision_check_model)
      ROS_INFO("    %s (%s)", collision_check_model->getName().c_str(), collision_check_model->getTypeId().c_str());
  }

  // get terrain model
  PluginManager::getPlugin(Instance()->terrain_model);
  if (Instance()->terrain_model)
  {
    ROS_INFO("[WorldModel] Found terrain model:");
    ROS_INFO("    %s (%s)", Instance()->terrain_model->getName().c_str(), Instance()->terrain_model->getTypeId().c_str());
  }
}

void WorldModel::loadParams(const ParameterSet& params)
{
  for (std::vector<CollisionCheckPlugin::Ptr>::iterator itr = Instance()->collision_check_models.begin(); itr != Instance()->collision_check_models.end(); itr++)
  {
    CollisionCheckPlugin::Ptr& collision_check_model = *itr;
    if (collision_check_model)
      collision_check_model->loadParams(params);
  }
}

bool WorldModel::isAccessible(const State& s)
{
  for (std::vector<CollisionCheckPlugin::Ptr>::const_iterator itr = Instance()->collision_check_models.begin(); itr != Instance()->collision_check_models.end(); itr++)
  {
    const CollisionCheckPlugin::Ptr& collision_check_model = *itr;
    if (collision_check_model && collision_check_model->isCollisionCheckAvailable() && !collision_check_model->isAccessible(s))
      return false;
  }
  return true;
}

bool WorldModel::isAccessible(const State& next, const State& current)
{
  for (std::vector<CollisionCheckPlugin::Ptr>::const_iterator itr = Instance()->collision_check_models.begin(); itr != Instance()->collision_check_models.end(); itr++)
  {
    const CollisionCheckPlugin::Ptr& collision_check_model = *itr;
    if (collision_check_model && collision_check_model->isCollisionCheckAvailable() && !collision_check_model->isAccessible(next, current))
      return false;
  }
  return true;
}

bool WorldModel::isTerrainModelAvailable()
{
  return Instance()->terrain_model && Instance()->terrain_model->isTerrainModelAvailable();
}

TerrainModelPlugin::Ptr WorldModel::getTerrainModel()
{
  return Instance()->terrain_model;
}

bool WorldModel::getHeight(double x, double y, double& height)
{
  if (!isTerrainModelAvailable())
    return true;

  return Instance()->terrain_model->getHeight(x, y, height);
}

bool WorldModel::update3DData(State& s)
{
  if (!isTerrainModelAvailable())
    return true;

  return Instance()->terrain_model->update3DData(s);
}
}
