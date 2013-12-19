#include "capability_map_generator/ReachabilityDummyInterface.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(capability_map_generator::ReachabilityDummyInterface, capability_map_generator::ReachabilityInterface)


namespace capability_map_generator
{

bool ReachabilityDummyInterface::isReachable(const octomap::pose6d & pose) const
{
    return true;
}

}

