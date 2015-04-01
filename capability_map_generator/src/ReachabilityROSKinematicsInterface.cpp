// Copyright (c) 2014, Jochen Kempfle
// All rights reserved.

/*
Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/


#include "capability_map_generator/ReachabilityROSKinematicsInterface.h"
#include "capability_map/CapabilityOcTreeNode.h"
#include <pluginlib/class_list_macros.h>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <kinematics_msgs/GetKinematicSolverInfo.h>
// #include <arm_kinematics_constraint_aware/arm_kinematics_constraint_aware_utils.h>
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

    std::string group_name;
    if(!nhP.getParam("group_name", group_name))
    {
        ROS_ERROR("No group_name defined!");
        ros::shutdown();
        exit(1);
    }
    std::string base_name;
    if(!nhP.getParam("base_name", base_name))
    {
        ROS_ERROR("No base_name defined!");
        ros::shutdown();
        exit(1);
    }
    std::string tip_name;
    if(!nhP.getParam("tip_name", tip_name))
    {
        ROS_ERROR("No tip_name defined!");
        ros::shutdown();
        exit(1);
    }

    kinematics->initialize(group_name, base_name, tip_name, 0.01);
    ROS_ASSERT(kinematics.get() != NULL);

    // get solver info service string
    std::string ik_solver_info_service;
    if(!nhP.getParam("ik_solver_info_service", ik_solver_info_service))
    {
        ROS_ERROR("No ik_solver_info_service defined!");
        ros::shutdown();
        exit(1);
    }

    ros::NodeHandle nh;
    ros::ServiceClient solver_info_client = nh.serviceClient<kinematics_msgs::GetKinematicSolverInfo>(ik_solver_info_service);
    if(!solver_info_client.waitForExistence()) {
        ROS_ERROR("IK service does not exist as %s", solver_info_client.getService().c_str());
    }

    // service messages for ik_solver_info
    kinematics_msgs::GetKinematicSolverInfo::Request request;
    kinematics_msgs::GetKinematicSolverInfo::Response response;

    if(!solver_info_client.call(request, response))
    {
        ROS_ERROR("Could not call GetKinematicSolverInfo query service");
        ros::shutdown();
        exit(1);
        // seed.resize(7, 0.0);
    }
    else
    {
        seed.resize(response.kinematic_solver_info.joint_names.size());
        for(size_t i = 0; i < response.kinematic_solver_info.joint_names.size(); ++i)
        {
            seed[i] = (response.kinematic_solver_info.limits[i].min_position + response.kinematic_solver_info.limits[i].max_position) / 2.0;
        }
    }
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

    return kinematics->searchPositionIK(ikPose, seed, 0.5, sol, err);
}

} // namespace

