#include "capability_map_generator/ReachabilityROSKinematicsInterface.h"
#include "capability_map/CapabilityOcTreeNode.h"
#include <pluginlib/class_list_macros.h>
#include <pluginlib/class_loader.h>
#include <cmath>

PLUGINLIB_EXPORT_CLASS(capability_map_generator::ReachabilityROSKinematicsInterface, capability_map_generator::ReachabilityInterface)

static pluginlib::ClassLoader<kinematics::KinematicsBase>* s_Kinematics = NULL;

boost::shared_ptr<kinematics::KinematicsBase> loadKinematics(ros::NodeHandle & nh)
{
    try {
        // create here with new as it can't go out of scope
        s_Kinematics
            = new pluginlib::ClassLoader<kinematics::KinematicsBase>
            ("kinematics_base", "kinematics::KinematicsBase");
    } catch(pluginlib::PluginlibException & ex) {
        // possible reason for failure: no known plugins
        ROS_ERROR("Could not instantiate class loader for kinematics::KinematicsBase - are there plugins registered? Error: %s", ex.what());
        return boost::shared_ptr<kinematics::KinematicsBase>();
    }

    std::string interface_name;
    if(!nh.getParam("kinematics_interface", interface_name)) {
        ROS_ERROR("No KinematicsBase defined!");
        return boost::shared_ptr<kinematics::KinematicsBase>();
    }
    ROS_INFO("Loading KinematicsBase %s", interface_name.c_str());
    boost::shared_ptr<kinematics::KinematicsBase> ri;
    try {
        ri = s_Kinematics->createInstance(interface_name);
    } catch(pluginlib::PluginlibException & ex) {
        ROS_ERROR("Failed to load KinematicsBaseinstance for: %s. Error: %s.",
                interface_name.c_str(), ex.what());
        return boost::shared_ptr<kinematics::KinematicsBase>();
    }

    return ri;
}


namespace capability_map_generator
{

ReachabilityROSKinematicsInterface::ReachabilityROSKinematicsInterface()
{
    ros::NodeHandle nhP("~");
    kinematics = loadKinematics(nhP);
    kinematics->initialize("left_arm",
            "base_link", "l_gripper_tool_frame", 0.01);
    ROS_ASSERT(kinematics.get() != NULL);

    for(int i = 0; i < 7; i++)
        seed.push_back(0.0);
}

bool ReachabilityROSKinematicsInterface::isReachable(const octomath::Pose6D &pose) const
{
    geometry_msgs::Pose ikPose;
    ikPose.position.x = pose.x();
    ikPose.position.y = pose.y();
    ikPose.position.z = pose.z();
    ikPose.orientation.x = pose.rot().x();
    ikPose.orientation.y = pose.rot().y();
    ikPose.orientation.z = pose.rot().z();
    ikPose.orientation.w = pose.rot().u();

    std::vector<double> sol;
    int err;

    return kinematics->getPositionIK(ikPose, seed, sol, err);
}

ReachabilityInterface::BoundingBox ReachabilityROSKinematicsInterface::getBoundingBox() const
{
    return ReachabilityInterface::BoundingBox(Vector(2.3, 0.1, 0.7), Vector(-0.1, -0.1, 0.3));
}

}

