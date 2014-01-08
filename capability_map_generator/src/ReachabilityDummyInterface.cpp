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
    if (pose.y() > -0.05 && pose.y() < 0.05 && pose.z() > 0.45 && pose.z() < 0.55)
    {
        if (pose.x() > 0.15 && pose.x() < 0.25)
        {
            cap = Capability(SPHERE, 0.0, 0.0, 0.0);
        }
        else if (pose.x() > 0.35 && pose.x() < 0.45)
        {
            cap = Capability(CONE, 0.0, 90.0, 10.0);
        }
        else if (pose.x() > 0.55 && pose.x() < 0.65)
        {
            cap = Capability(CONE, 90.0, 0.0, 45.0);
        }
        else if (pose.x() > 0.75 && pose.x() < 0.85)
        {
            cap = Capability(CONE, 40.0, 20.0, 100.0);
        }
        else if (pose.x() > 0.95 && pose.x() < 1.05)
        {
            cap = Capability(CONE, 180.0, 120.0, 70.0);
        }
        else if (pose.x() > 1.15 && pose.x() < 1.25)
        {
            cap = Capability(CYLINDER_1, 0.0, 0.0, 10.0);
        }
        else if (pose.x() > 1.35 && pose.x() < 1.45)
        {
            cap = Capability(CYLINDER_1, 0.0, 0.0, 90.0);
        }
        else if (pose.x() > 1.55 && pose.x() < 1.65)
        {
            cap = Capability(CYLINDER_1, 40.0, 50.0, 45.0);
        }
        else if (pose.x() > 1.75 && pose.x() < 1.85)
        {
            cap = Capability(CYLINDER_2, 0.0, 0.0, 10.0);
        }
        else if (pose.x() > 1.95 && pose.x() < 2.05)
        {
            cap = Capability(CYLINDER_2, 0.0, 0.0, 45.0);
        }
        else if (pose.x() > 2.15 && pose.x() < 2.25)
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

ReachabilityInterface::BoundingBox ReachabilityDummyInterface::getBoundingBox() const
{
    return ReachabilityInterface::BoundingBox(Vector(2.3, 0.1, 0.7), Vector(-0.1, -0.1, 0.3));
}

}

