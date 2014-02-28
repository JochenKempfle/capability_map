#include "capability_map_generator/ReachabilityDummyInterface.h"
#include "capability_map/CapabilityOcTreeNode.h"
#include <pluginlib/class_list_macros.h>
#include <cmath>

PLUGINLIB_EXPORT_CLASS(capability_map_generator::ReachabilityDummyInterface, capability_map_generator::ReachabilityInterface)


namespace capability_map_generator
{

bool ReachabilityDummyInterface::isReachable(const octomath::Pose6D &pose) const
{
    Capability cap;
    if (pose.y() > 0.04 && pose.y() < 0.06 && pose.z() > 0.44 && pose.z() < 0.46)
    {
        if (pose.x() > 0.14 && pose.x() < 0.16)
        {
            cap = Capability(SPHERE, 0.0, 0.0, 0.0);
        }
        else if (pose.x() > 0.34 && pose.x() < 0.36)
        {
            cap = Capability(CONE, 0.0, 90.0, 15.0);
        }
        else if (pose.x() > 0.54 && pose.x() < 0.56)
        {
            cap = Capability(CONE, 90.0, 0.0, 45.0);
        }
        else if (pose.x() > 0.74 && pose.x() < 0.76)
        {
            cap = Capability(CONE, 90.0, 0.0, 125.0);
        }
        else if (pose.x() > 0.94 && pose.x() < 0.96)
        {
            cap = Capability(CONE, 180.0, 120.0, 70.0);
        }
        else if (pose.x() > 1.14 && pose.x() < 1.16)
        {
            cap = Capability(CYLINDER_1, 0.0, 0.0, 10.0);
        }
        else if (pose.x() > 1.34 && pose.x() < 1.36)
        {
            cap = Capability(CYLINDER_1, 0.0, 0.0, 90.0);
        }
        else if (pose.x() > 1.54 && pose.x() < 1.56)
        {
            cap = Capability(CYLINDER_1, 40.0, 50.0, 45.0);
        }
        else if (pose.x() > 1.74 && pose.x() < 1.76)
        {
            cap = Capability(CYLINDER_2, 0.0, 0.0, 10.0);
        }
        else if (pose.x() > 1.94 && pose.x() < 1.96)
        {
            cap = Capability(CYLINDER_2, 0.0, 0.0, 45.0);
        }
        else if (pose.x() > 2.14 && pose.x() < 2.16)
        {
            cap = Capability(CYLINDER_2, 90.0, 90.0, 89.0);
        }
        else
        {
            return false;
        }
    }
    else
    {
        return false;
    }

    octomath::Vector3 unitVector(1.0, 0.0, 0.0);
    octomath::Vector3 rotatedVector = pose.rot().rotate(unitVector);

    double phi = atan2(rotatedVector.y(), rotatedVector.x()) * 180.0 / M_PI;
    double theta = acos(rotatedVector.z()) * 180.0 / M_PI;

    return cap.isDirectionPossible(phi, theta);
}

} // namespace


