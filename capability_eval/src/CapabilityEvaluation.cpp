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

octomath::Quaternion vectorToQuaternion(octomath::Vector3 vec)
{
    vec.normalize();
    octomath::Vector3 originVector(1.0, 0.0, 0.0);
    
    octomath::Vector3 c = originVector.cross(vec);
    
    octomath::Quaternion quaternion(1 + originVector.dot(vec), c.x(), c.y(), c.z());
    
    return quaternion.normalize();
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "capability_eval");

    ros::NodeHandle nhPriv("~");

    // load the reachability interface given by launch file
    boost::shared_ptr<capability_map_generator::ReachabilityInterface> ri = loadReachabilityInterface(nhPriv);

    // arguments
    TCLAP::CmdLine cmd("Evaluates the capability map given by argument", ' ', "1.0");

    std:: string msg = "Path to the capability map to be evaluated.\nExample: -p mydir/mysubdir/filename.cpm";
    TCLAP::ValueArg<std::string> pathNameArg("p", "path", msg, true, "./capability_map.cpm", "string");

    msg = "Number of samples. Default is 10000.";
    TCLAP::ValueArg<unsigned int> numSamplesArg("n", "numSamples", msg, false, 10000, "integer");

    msg = "Sets another seed. Default is 1.";
    TCLAP::ValueArg<unsigned int> seedArg("s", "seed", msg, false, 1, "integer");

    cmd.add(seedArg);
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
        ros::shutdown();
        exit(1);
    }

    const size_t numSamples = numSamplesArg.getValue();

    // load the capability map
    std::string pathName = pathNameArg.getValue();
    CapabilityOcTree* tree = CapabilityOcTree::readFile(pathName);

    if (tree == NULL)
    {
        printf("could not read file %s, check if you used an absolute path to the file, roslaunch usually changes the working directory\n", pathName.c_str());
        ros::shutdown();
        exit(1);
    }

    // set the seed
    srand(seedArg.getValue());

    std::vector<octomap::pose6d> poses(numSamples);

    std::vector<bool> nativePossibilities(numSamples), capabilityPossibilities(numSamples);

    printf("Generating %d random poses. Bounding box is (-1, -1, -1) to (1, 1, 1)\n", numSamples);

    for (size_t i = 0; i < numSamples; ++i)
    {
        double a = double(rand() % 2001) / 1000.0 - 1.0;
        double b = double(rand() % 2001) / 1000.0 - 1.0;
        double c = double(rand() % 2001) / 1000.0 - 1.0;
        double x = double(rand() % 2001) / 1000.0 - 1.0;
        double y = double(rand() % 2001) / 1000.0 - 1.0;
        double z = double(rand() % 2001) / 1000.0 - 1.0;

        poses[i] = octomap::pose6d(octomath::Vector3(x, y, z), vectorToQuaternion(octomath::Vector3(a, b, c)));
    }

    printf("done\nStarting evaluation of trying to reach poses the native way\n");

    // try to reach 10000 random poses, count the time

    ros::Time startTimeNative = ros::Time::now();
    for (size_t i = 0; i < numSamples; ++i)
    {
        if (ri->isReachable(poses[i]))
        {
            nativePossibilities[i] = true;
        }
        else
        {
            nativePossibilities[i] = false;
        }
    }
    ros::Time endTimeNative = ros::Time::now();

    printf("done\nStarting evaluation of trying to reach poses with capability map\n");

    ros::Time startTimeCapability = ros::Time::now();
    for (size_t i = 0; i < numSamples; ++i)
    {
        if (tree->isPosePossible(poses[i]))
        {
            capabilityPossibilities[i] = true;
        }
        else
        {
            capabilityPossibilities[i] = false;
        }
    }
    ros::Time endTimeCapability = ros::Time::now();

    printf("done\n\n");

    unsigned int numSuccessful = 0;
    unsigned int numWrongPositives = 0;
    unsigned int numWrongNegatives = 0;

    for (size_t i = 0; i < numSamples; ++i)
    {
        if (capabilityPossibilities[i] == nativePossibilities[i])
        {
            numSuccessful++;
        }
        else
        {
            if (capabilityPossibilities[i] == true)
            {
                numWrongPositives++;
            }
            else
            {
                numWrongNegatives++;
            }
        }
    }

    unsigned int numNotEmptyCaps = 0;

    // loop through all capabilities and count how much of them are not empty
    for (CapabilityOcTree::leaf_iterator it = tree->begin_leafs(), end = tree->end_leafs(); it != end; ++it)
    {
        if (it->getCapability().getType() != EMPTY)
        {
            numNotEmptyCaps++;
        }
    }

    int secsNative = int((endTimeNative - startTimeNative).toSec());
    int minsNative = secsNative / 60;
    int mSecsNative = int((endTimeNative - startTimeNative).toSec() * 1000.0) % 1000;
    secsNative %= 60;
    printf("Time passed for native: %d min %d sec %d msec\n", minsNative, secsNative, mSecsNative);

    int secsCapability = int((endTimeCapability - startTimeCapability).toSec());
    int minsCapability = secsCapability / 60;
    int mSecsCapability = int((endTimeCapability - startTimeCapability).toSec() * 1000.0) % 1000;
    int mySecsCapability = int((endTimeCapability - startTimeCapability).toSec() * 1000000.0) % 1000;
    secsCapability %= 60;
    printf("Time passed for capabilities: %d min %d sec %d msec %d mysec\n\n", minsCapability,
                                          secsCapability, mSecsCapability, mySecsCapability);

    printf("Resolution of capability map is: %g\n", tree->getResolution());
    printf("Number of not empty capabilities: %d\n", numNotEmptyCaps);
    printf("Number of samples: %d\nNumber of correct estimated poses: %d\n", numSamples, numSuccessful);
    printf("Number of not correct estimated poses: %d\n", numWrongPositives + numWrongNegatives);
    printf("Number of wrong positives: %d\nNumber of wrong negatives: %d\n", numWrongPositives, numWrongNegatives);
    printf("Correctness of capability map in percent: %3.2f\n\n", 100.0 * double(numSuccessful) / double(numSamples));

    printf("Memory usage of a single node in bytes: %d\n", tree->memoryUsageNode());
    printf("Main memory usage of the complete map in bytes: %d\n", tree->memoryUsage());
}






