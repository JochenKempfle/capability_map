#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <tclap/CmdLine.h>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <string>
#include "capability_map_generator/ReachabilityInterface.h"
#include "capability_map/CapabilityOcTree.h"


static pluginlib::ClassLoader<capability_map_generator::ReachabilityInterface>* s_ReachbilityInterface = NULL;

boost::shared_ptr<capability_map_generator::ReachabilityInterface> loadReachabilityInterface(ros::NodeHandle &nh)
{
    try {
        // create here with new as it can't go out of scope
        s_ReachbilityInterface
            = new pluginlib::ClassLoader<capability_map_generator::ReachabilityInterface>
            ("capability_map_generator", "capability_map_generator::ReachabilityInterface");
    } catch(pluginlib::PluginlibException & ex) {
        // possible reason for failure: no known plugins
        ROS_ERROR("Could not instantiate class loader for capability_map_generator::ReachabilityInterface - are there plugins registered? Error: %s", ex.what());
        return boost::shared_ptr<capability_map_generator::ReachabilityInterface>();
    }

    std::string interface_name;
    if(!nh.getParam("reachability_interface", interface_name)) {
        ROS_ERROR("No ReachabilityInterface defined!");
        return boost::shared_ptr<capability_map_generator::ReachabilityInterface>();
    }
    ROS_INFO("Loading ReachabilityInterface %s", interface_name.c_str());
    boost::shared_ptr<capability_map_generator::ReachabilityInterface> ri;
    try {
        ri = s_ReachbilityInterface->createInstance(interface_name);
    } catch(pluginlib::PluginlibException & ex) {
        ROS_ERROR("Failed to load ReachabilityInterface instance for: %s. Error: %s.",
                interface_name.c_str(), ex.what());
        return boost::shared_ptr<capability_map_generator::ReachabilityInterface>();
    }

    return ri;
}

octomath::Quaternion vectorToQuaternion(const octomath::Vector3 &vec)
{
    vec.normalize();

    octomath::Vector3 originVector(1.0, 0.0, 0.0);
    
    octomath::Vector3 c = originVector.cross(vec);
    
    octomath::Quaternion quaternion(1 + originVector.dot(vec), c.x, c.y, c.z);
    
    return quaternion.normalize();
}


int main(int argc, char** argv)
{
    // load the reachability interface given by launch file
    boost::shared_ptr<capability_map_generator::ReachabilityInterface> ri = loadReachabilityInterface(nhPriv);

    // arguments
    TCLAP::CmdLine cmd("Evaluates the capability map given by argument", ' ', "1.0");

    std:: string msg = "Path to the capability map to be evaluated.\nExample: -p mydir/mysubdir/filename.cpm";
    TCLAP::ValueArg<std::string> pathNameArg("p", "path", msg, true, "./capability_map.cpm", "string");

    msg = "Number of samples. Default is 10000.";
    TCLAP::ValueArg<unsigned int> numSamplesArg("n", "numSamples", msg, false, 10000, "integer");

    cmd.add(numSamplesArg);
    cmd.add(pathNameArg);

    // parse arguments with TCLAP
    try
    {
        cmd.parse(argc, argv);
    }
    catch (TCLAP::ArgException &e)  // catch any exceptions
    {
        printf("Error: %s for argument %s\n", e.error().c_str(), e.argId().c_str());
        //ros::shutdown();
        exit(1);
    }

    const size_t numSamples = numSamplesArg.getValue();
    std::string pathName = pathNameArg.getValue();

    CapabilityOcTree* tree = CapabilityOcTree::readFile(pathName);

    srand(12345);

    std::vector<octomap::pose6d> poses(numSamples);

    std::vector<bool> nativePossibilities(numSamples), capabilityPossibilities(numSamples);

    for (int i = 0; i < numSamples; ++i)
    {
        double a = double(rand() % 2001) / 1000.0 - 1.0;
        double b = double(rand() % 2001) / 1000.0 - 1.0;
        double c = double(rand() % 2001) / 1000.0 - 1.0;
        double x = double(rand() % 2001) / 1000.0 - 1.0;
        double y = double(rand() % 2001) / 1000.0 - 1.0;
        double z = double(rand() % 2001) / 1000.0 - 1.0;

        poses[i] = octomap::pose6d(octomath::Vector3(x, y, z), vectorToQuaternion(octomath::Vector3(a, b, c)));
    }

    // try to reach 10000 random poses, count the time

    ros::Time startTimeNative = ros::Time::now();
    for (size_t i = 0; i < numSamples; ++i)
    {
        if (ri->isReachable(poses[i])
        {
            nativePossibilities[i] = true;
        }
        else
        {
            nativePossibilities[i] = false;
        }
    }
    ros::Time endTimeNative = ros::Time::now();

    ros::Time startTimeCapability = ros::Time::now();
    for (size_t i = 0; i < numSamples; ++i)
    {
        if (tree->isPosePossible(poses[i])
        {
            capabilityPossibilities[i] = true;
        }
        else
        {
            capabilityPossibilities[i] = false;
        }
    }
    ros::Time endTimeCapability = ros::Time::now();

    int secsNative = int((endTimeNative - startTimeNative).toSec());
    int minsNative = secs / 60;
    secs %= 60;
    printf("Time passed for native: %d min %d sec\n", mins, secs);

    int secsCapability = int((endTimeCapability - startTimeCapability).toSec());
    int minsCapability = secs / 60;
    secs %= 60;
    printf("Time passed for capabilities: %d min %d sec\n", mins, secs);
}






