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


#include <ros/ros.h>
#include <tclap/CmdLine.h>
#include <cmath>
#include <octomap/octomap.h>
#include "capability_map/CapabilityOcTree.h"
#include "capability_map/CapabilityOcTreeNode.h"
#include "capability_map/Reachability_srv.h"

CapabilityOcTree* tree = NULL;

bool isReachable(capability_map::Reachability_srv::Request &req, capability_map::Reachability_srv::Response &res)
{
    if (tree == NULL)
    {
        return false;
    }
    octomath::Quaternion orientation(req.pose.orientation.w, req.pose.orientation.x, req.pose.orientation.y, req.pose.orientation.z);
    
    octomath::Vector3 unitVector(1.0, 0.0, 0.0);
    octomath::Vector3 rotatedVector = orientation.rotate(unitVector);

    double phi = atan2(rotatedVector.y(), rotatedVector.x()) * 180.0 / M_PI;
    double theta = acos(rotatedVector.z()) * 180.0 / M_PI;
    
    Capability cap = tree->getNodeCapability(req.pose.position.x, req.pose.position.y, req.pose.position.z);
    
    res.reachable = cap.isDirectionPossible(phi, theta);
    res.SFE = cap.getShapeFitError();
    
    return true;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "capability_map_server");
    ros::NodeHandle n;
  
   // arguments
    TCLAP::CmdLine cmd("Loads the capability map given by argument", ' ', "1.0");

    TCLAP::ValueArg<std::string> pathNameArg("p", "path", "path and filename of the capability map to be loaded.\n\
                                             Example: -p mydir/mysubdir/filename.cpm", true, "./capability_map.cpm", "string");

    cmd.add(pathNameArg);

    // parse arguments with TCLAP
    try
    {
        cmd.parse(argc, argv);
    }
    catch (TCLAP::ArgException &e)  // catch any exceptions
    {
        ROS_ERROR("Error: %s for argument %s", e.error().c_str(), e.argId().c_str());
        exit(1);
    }

    std::string pathName = pathNameArg.getValue();

    tree = CapabilityOcTree::readFile(pathName);

    if (tree == NULL)
    {
        ROS_ERROR("Could not load capability map in file %s", pathName.c_str());
        ros::shutdown();
        exit(1);
    }

    ros::ServiceServer service = n.advertiseService("capability_map", isReachable);
  
    ros::spin();

    return 0;
}
