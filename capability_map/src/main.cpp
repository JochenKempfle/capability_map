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

    tree = dynamic_cast<CapabilityOcTree*>(AbstractOcTree::read(pathName));

    ros::ServiceServer service = n.advertiseService("capability_map", isReachable);
  
    ros::spin();

    return 0;
}
