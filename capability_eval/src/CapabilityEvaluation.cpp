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


#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <tclap/CmdLine.h>
#include <sstream>
#include <cstdio>
#include <ostream>
#include <fstream>
#include <cstdlib>
#include <vector>
#include <string>
#include <utility>
#include <cmath>
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

    msg = "If set, writes a log file containing time required and number of computed capabilities to map_name.cpm.eval_log";
    TCLAP::SwitchArg logArg("l", "log", msg, false);

    cmd.add(seedArg);
    cmd.add(logArg);
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
    bool loggingEnabled = logArg.getValue();

    // load the capability map
    std::string pathName = pathNameArg.getValue();
    CapabilityOcTree* tree = CapabilityOcTree::readFile(pathName);

    if (tree == NULL)
    {
        printf("Could not read file %s, check if you used an absolute path to the file, roslaunch usually changes the working directory\n", pathName.c_str());
        ros::shutdown();
        exit(1);
    }

    // set the seed
    srand(seedArg.getValue());

    std::vector<octomap::pose6d> poses(numSamples);

    std::vector<bool> nativePossibilities(numSamples), capabilityPossibilities(numSamples);

    printf("Generating %d random poses lying in the workspace of the robot.\n", numSamples);

    // generate random poses
    for (size_t i = 0; i < numSamples; ++i)
    {
        double x = double(rand() % 2001) / 1000.0 - 1.0;
        double y = double(rand() % 2001) / 1000.0 - 1.0;
        double z = double(rand() % 2001) / 1000.0 - 1.0;
        // if the capability at given position is empty search a new one
        if (tree->getNodeCapability(x, y, z).getType() == EMPTY)
        {
            --i;
            continue;
        }
        double a = double(rand() % 2001) / 1000.0 - 1.0;
        double b = double(rand() % 2001) / 1000.0 - 1.0;
        double c = double(rand() % 2001) / 1000.0 - 1.0;

        poses[i] = octomap::pose6d(octomath::Vector3(x, y, z), vectorToQuaternion(octomath::Vector3(a, b, c)));
    }

    printf("done\nStart evaluation of trying to reach poses the native way\n");

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

    printf("done\nStart evaluation of trying to reach poses with capability map\n");

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

    unsigned int numTruePositives = 0;
    unsigned int numTrueNegatives = 0;
    unsigned int numFalsePositives = 0;
    unsigned int numFalseNegatives = 0;

    for (size_t i = 0; i < numSamples; ++i)
    {
        if (capabilityPossibilities[i] == nativePossibilities[i])
        {
            if (capabilityPossibilities[i] == true)
            {
                numTruePositives++;
            }
            else
            {
                numTrueNegatives++;
            }
        }
        else
        {
            if (capabilityPossibilities[i] == true)
            {
                numFalsePositives++;
            }
            else
            {
                numFalseNegatives++;
            }
        }
    }

    unsigned int numNotEmptyCaps = 0;
    std::vector<int> shapeFitErrorTable(101, 0);

    // loop through all capabilities and count how much of them are not empty and create SFE table
    for (CapabilityOcTree::leaf_iterator it = tree->begin_leafs(), end = tree->end_leafs(); it != end; ++it)
    {
        if (it->getCapability().getType() != EMPTY)
        {
            numNotEmptyCaps++;
            shapeFitErrorTable[int(round(it->getCapability().getShapeFitError()))]++;
        }
    }

    // get time used by native
    int secsNative = int((endTimeNative - startTimeNative).toSec());
    int minsNative = secsNative / 60;
    int mSecsNative = int((endTimeNative - startTimeNative).toSec() * 1000.0) % 1000;
    secsNative %= 60;

    // get time used by capability map
    int secsCapability = int((endTimeCapability - startTimeCapability).toSec());
    int minsCapability = secsCapability / 60;
    int mSecsCapability = int((endTimeCapability - startTimeCapability).toSec() * 1000.0) % 1000;
    int mySecsCapability = int((endTimeCapability - startTimeCapability).toSec() * 1000000.0) % 1000;
    secsCapability %= 60;

    std::vector<std::pair<size_t, ros::Duration> > numAndTimeMinReachPercent;

    // count time and num of positions for getPositionsWithMinReachablePercent()
    for (int i = 0; i < 101; i += 5)
    {
        ros::Time start = ros::Time::now();
        size_t num = tree->getPositionsWithMinReachablePercent(double(i)).size();
        ros::Time end = ros::Time::now();
        numAndTimeMinReachPercent.push_back(std::make_pair(num, end - start));
    }

    // output to screen/log file
    std::ostringstream log;

    log << "Resolution of capability map is: " << tree->getResolution() << std::endl;
    log << "Group name is: " << tree->getGroupName() << std::endl;
    log << "Base name is: " << tree->getBaseName() << std::endl;
    log << "Tip name is: " << tree->getTipName() << std::endl << std::endl;

    log << "Number of not empty capabilities: " << numNotEmptyCaps << std::endl << std::endl;

    log << "Time passed for native: " << minsNative << " min " << secsNative << " sec " << mSecsNative << " msec" << std::endl;
    log << "Time passed for capability: " << minsCapability << " min " << secsCapability << " sec ";
    log << mSecsCapability << " msec " << mySecsCapability << " mysec" << std::endl << std::endl;

    log << "Number of samples: " << numSamples << std::endl;
    log << "Number of correct estimated poses: " << (numTruePositives + numTrueNegatives) << std::endl;
    log << "Number of not correct estimated poses: " << (numFalsePositives + numFalseNegatives) << std::endl;
    log << "Correctness of capability map: ";
    log << (100.0 * double(numTruePositives + numTrueNegatives) / double(numSamples)) << "%" << std::endl << std::endl;

    log << "Number of true positives: " << numTruePositives;
    log << " (" << 100.0 * double(numTruePositives) / double(numSamples) << "%)" << std::endl;
    log << "Number of true negatives: " << numTrueNegatives;
    log << " (" << 100.0 * double(numTrueNegatives) / double(numSamples) << "%)" << std::endl;
    log << "Number of false positives: " << numFalsePositives;
    log << " (" << 100.0 * double(numFalsePositives) / double(numSamples) << "%)" << std::endl;
    log << "Number of false negatives: " << numFalseNegatives;
    log << " (" << 100.0 * double(numFalseNegatives) / double(numSamples) << "%)" << std::endl;
    log << std::endl;

    log << "Evaluation of getPositionsWithMinReachablePercent():" << std::endl;
    for (size_t i = 0; i < numAndTimeMinReachPercent.size(); ++i)
    {
        log << "Min percent reachable: " << i * 100.0 / (numAndTimeMinReachPercent.size() - 1);
        log << "% Count: " << numAndTimeMinReachPercent[i].first;
        int secs = int(numAndTimeMinReachPercent[i].second.toSec()) % 60;
        int mSecs = int(numAndTimeMinReachPercent[i].second.toSec() * 1000.0) % 1000;
        int mySecs = int(numAndTimeMinReachPercent[i].second.toSec() * 1000000.0) % 1000;
        log << " Time: " << secs << "sec " << mSecs << " msec " << mySecs << " mysec" << std::endl;
    }
    log << std::endl;

    log << "Shape Fit Errors:" << std::endl;
    for (size_t i = 0; i < shapeFitErrorTable.size(); ++i)
    {
        if (shapeFitErrorTable[i] == 0)
        {
            continue;
        }
        log << "SFE: " << i << " count: " << shapeFitErrorTable[i];
        log << " (" << 100.0 * double(shapeFitErrorTable[i]) / double(numNotEmptyCaps) << "%)" << std::endl;
    }
    log << std::endl;

    log << "Memory usage of a single node in bytes: " << tree->memoryUsageNode() << std::endl;
    log << "Main memory usage of the complete map in bytes: " << tree->memoryUsage() << std::endl;

    std::cout << log.str() << std::endl;

    if (loggingEnabled)
    {
        std::string pathNameLog = pathName + ".eval_log";
        std::ofstream file(pathNameLog.c_str());
        if (file.is_open())
        {
            file << log.str();
            std::cout << "Written log file to " << pathNameLog << std::endl;
        }
        else
        {
            std::cout << "Error: Could not open " << pathNameLog << ", no log file written." << std::endl;
        }
    }
}






