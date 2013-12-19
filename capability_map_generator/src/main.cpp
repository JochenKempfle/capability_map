#include "capability_map_generator/ReachabilityInterface.h"
#include <pluginlib/class_loader.h>
#include <ros/ros.h>

static pluginlib::ClassLoader<capability_map_generator::ReachabilityInterface>* s_ReachbilityInterface = NULL;

boost::shared_ptr<capability_map_generator::ReachabilityInterface> loadPlanner(ros::NodeHandle & nh)
{
    try {
        // create here with new as it can't go out of scope
        s_ReachbilityInterface
            = new pluginlib::ClassLoader<capability_map_generator::ReachabilityInterface>
            ("capability_map_generator", "capability_map_generator::ReachabilityInterface");
    } catch(pluginlib::PluginlibException & ex) {
        // possible reason for failure: no known plugins
        ROS_ERROR("Could not instantiate class loader for continual_planning_executive::PlannerInterface - are there plugins registered? Error: %s", ex.what());
        return boost::shared_ptr<capability_map_generator::ReachabilityInterface>();
    }

    std::string interface_name;
    if(!nh.getParam("reachability_interface", interface_name)) {
        ROS_ERROR("No planner defined!");
        return boost::shared_ptr<capability_map_generator::ReachabilityInterface>();
    }
    ROS_INFO("Loading planner %s", interface_name.c_str());
    boost::shared_ptr<capability_map_generator::ReachabilityInterface> ri;
    try {
        ri = s_ReachbilityInterface->createInstance(interface_name);
    } catch(pluginlib::PluginlibException & ex) {
        ROS_ERROR("Failed to load Planner instance for: %s. Error: %s.",
                interface_name.c_str(), ex.what());
        return boost::shared_ptr<capability_map_generator::ReachabilityInterface>();
    }

    return ri;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "capability_map_generator");

    ros::NodeHandle nhPriv("~");

    boost::shared_ptr<capability_map_generator::ReachabilityInterface> ri = loadPlanner(nhPriv);

    ROS_ASSERT(ri);

}

