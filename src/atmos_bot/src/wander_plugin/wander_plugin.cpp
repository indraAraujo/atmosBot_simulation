#include "wander_plugin_header"

namespace atmos_bot
{

WanderPlugin::WanderPlugin(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<atmos_bot::action::Wander>(xml_tag_name, conf)
{
}

void WanderPlugin::on_tick()
{
  atmos_bot::action::Wander::Goal goal;
  goal.start_wandering = true;
  this->set_goal(goal);
}

BT::NodeStatus WanderPlugin::on_success()
{

  return BT::NodeStatus::SUCCESS;
}

} 

BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder = [](const std::string& name, const BT::NodeConfiguration& config) {
    return std::make_unique<atmos_bot::WanderPlugin>(name, config);
  };

  factory.registerBuilder<atmos_bot::WanderPlugin>("WanderPlugin", builder);
}
