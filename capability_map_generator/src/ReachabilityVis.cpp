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


#include "capability_map_generator/ReachabilityInterface.h"
#include "capability_map_generator/Vector.h"
#include "capability_map_generator/ReachabilitySphere.h"
#include "capability_map/CapabilityOcTree.h"
#include <visualization_msgs/MarkerArray.h>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <tclap/CmdLine.h>
#include <string>
#include <vector>
#include <utility>
#include <algorithm>


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

// function which equally distibutes points on a sphere
// (see http://web.archive.org/web/20120421191837/http://www.cgafaq.info/wiki/Evenly_distributed_points_on_sphere)
std::vector<capability_map_generator::Vector> distributePointsOnSphere(unsigned int n)
{
    // variant of spiral point algorithm (Saff et al.) which uses golden angle to spread points
    double goldenAngle = M_PI * (3.0 - sqrt(5.0));
    double dz = 2.0 / n;
    double angle = 0.0;
    double z = 1.0 - dz / 2.0;

    std::vector<capability_map_generator::Vector> points(n);

    for (unsigned int i = 0; i < n; ++i)
    {
        double r = sqrt(1.0 - z*z);
        points[i] = capability_map_generator::Vector(cos(angle) * r, sin(angle) * r, z);
        z = z - dz;
        angle = angle + goldenAngle;
    }

    return points;
}

octomath::Quaternion vectorToQuaternion(const capability_map_generator::Vector &vec)
{
    capability_map_generator::Vector originVector(1.0, 0.0, 0.0);
    
    capability_map_generator::Vector c = originVector.cross(vec);
    
    octomath::Quaternion quaternion(1 + originVector.dot(vec), c.x, c.y, c.z);
    
    return quaternion.normalize();
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "capability_map_generator");

    ros::NodeHandle nhPriv("~");
    
    // load the reachability interface given by launch file
    boost::shared_ptr<capability_map_generator::ReachabilityInterface> ri = loadReachabilityInterface(nhPriv);

    ROS_ASSERT(ri);
    
    std::string msg = "Sends reachability spheres of the region specified by given bounding box as MarkerArray to rviz.";
    TCLAP::CmdLine cmd(msg, ' ', "1.0");

    msg = "Number of samples per voxel. Default is 200.";
    TCLAP::ValueArg<unsigned int> numSamplesArg("n", "numSamples", msg, false, 200, "integer");

    msg = "Distance between two voxels in meter, default is 0.1 m.";
    TCLAP::ValueArg<double> resolutionArg("r", "resolution", msg, false, 0.1, "floating point");

    msg = "The start/end point of the bounding box in x-direction.\n\
           If only one x-value is given, a slice (or a point) at this position depending on y- and z-values gets computed.\n\
           If more than 2 values are given, the boundaries are from min(x1, x2, ...) to max(x1, x2, ...).\n\
           Example: -x -0.1 -x 2.3";
    TCLAP::MultiArg<double> xArg("x", "x-pos", msg, true, "floating point");

    msg = "The start/end point of the bounding box in y-direction.\n\
           If only one y-value is given, a slice (or a point) at this position depending on x- and z-values gets computed.\n\
           If more than 2 values are given, the boundaries are from min(y1, y2, ...) to max(y1, y2, ...).\n\
           Example: -y -0.1 -y 2.3";
    TCLAP::MultiArg<double> yArg("y", "y-pos", msg, true, "floating point");

    msg = "The start/end point of the bounding box in z-direction.\n\
           If only one z-value is given, a slice (or a point) at this position depending on x- and y-values gets computed.\n\
           If more than 2 values are given, the boundaries are from min(z1, z2, ...) to max(z1, z2, ...).\n\
           Example: -z -0.1 -z 2.3";
    TCLAP::MultiArg<double> zArg("z", "z-pos", msg, true, "floating point");

    cmd.add(zArg);
    cmd.add(yArg);
    cmd.add(xArg);
    cmd.add(resolutionArg);
    cmd.add(numSamplesArg);

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
    
    // get values from arguments
    int numSamples = numSamplesArg.getValue();
    double resolution = resolutionArg.getValue();
    
    // check if values are valid
    if (numSamples <= 0)
    {
        ROS_ERROR("Error: number of samples must be positive and greater than 0");
        exit(1);
    }
    if (resolution <= 0.0)
    {
        ROS_ERROR("Error: resolution must be positive and greater than 0.0");
        exit(1);
    }
    
    // get coordinates of equally distributed points over a sphere
    std::vector<capability_map_generator::Vector> spherePoints = distributePointsOnSphere(numSamples);
    
    // calculate the corresponding quaternions for the sphere point coordinates
    std::vector<octomath::Quaternion> quaternions(spherePoints.size());
    for (size_t i = 0; i < spherePoints.size(); ++i)
    {
        quaternions[i] = vectorToQuaternion(spherePoints[i]);
    }

    // a vector of ReachabilitySpheres at specific position for displaying
    std::vector<std::pair<capability_map_generator::Vector, capability_map_generator::ReachabilitySphere> > spheres;

    // get and adjust the boundaries for iteration
    /*
    capability_map_generator::ReachabilityInterface::BoundingBox bbx = ri->getBoundingBox();

    double startX = bbx.getStartPoint().x < bbx.getEndPoint().x ? bbx.getStartPoint().x : bbx.getEndPoint().x;
    double endX = bbx.getStartPoint().x < bbx.getEndPoint().x ? bbx.getEndPoint().x : bbx.getStartPoint().x;
    double startY = bbx.getStartPoint().y < bbx.getEndPoint().y ? bbx.getStartPoint().y : bbx.getEndPoint().y;
    double endY = bbx.getStartPoint().y < bbx.getEndPoint().y ? bbx.getEndPoint().y : bbx.getStartPoint().y;
    double startZ = bbx.getStartPoint().z < bbx.getEndPoint().z ? bbx.getStartPoint().z : bbx.getEndPoint().z;
    double endZ = bbx.getStartPoint().z < bbx.getEndPoint().z ? bbx.getEndPoint().z : bbx.getStartPoint().z;
    */

    std::vector<double> xValues = xArg.getValue();
    std::vector<double> yValues = yArg.getValue();
    std::vector<double> zValues = zArg.getValue();

    std::sort(xValues.begin(), xValues.end());
    std::sort(yValues.begin(), yValues.end());
    std::sort(zValues.begin(), zValues.end());

    // create a dummy tree to get alignment
    CapabilityOcTree tree(resolution);

    // get and adjust the boundaries for iteration (add a small value to end due to floating point precision)
    double startX = tree.getAlignment(xValues[0]);
    double endX = tree.getAlignment(xValues[xValues.size() - 1]) + resolution/100.0;
    double startY = tree.getAlignment(yValues[0]);
    double endY = tree.getAlignment(yValues[yValues.size() - 1]) + resolution/100.0;
    double startZ = tree.getAlignment(zValues[0]);
    double endZ = tree.getAlignment(zValues[zValues.size() - 1]) + resolution/100.0;

    double numCapsToCompute = ((endX - startX) / resolution + 1.0) * ((endY - startY) / resolution + 1.0)
                              * ((endZ - startZ) / resolution + 1.0);
    double numCapsComputed = 0.0;

    ROS_INFO("Number of capabilities to compute: %d", (unsigned int)numCapsToCompute);

    // progress in percent
    double progress = 0.0;
    double progressLimiter = 0.0;

    capability_map_generator::ReachabilitySphere sphere;

    for(double x = startX; x <= endX; x += resolution)
    {
        for(double y = startY; y <= endY; y += resolution)
        {
            for(double z = startZ; z <= endZ; z += resolution)
            {
                for (int i = 0; i < numSamples; ++i)
                {
                    octomap::pose6d pose(octomath::Vector3(x, y, z), quaternions[i]);
                    if (ri->isReachable(pose))
                    {
                        sphere.appendDirection(spherePoints[i].x, spherePoints[i].y, spherePoints[i].z, true);
                    }
                    else
                    {
                        sphere.appendDirection(spherePoints[i].x, spherePoints[i].y, spherePoints[i].z, false);
                    }
                }
                if (!ros::ok())
                {
                    ros::shutdown();
                    exit(1);
                }
                spheres.push_back(std::make_pair(capability_map_generator::Vector(x, y, z), sphere));
                sphere.clear();

                numCapsComputed += 1.0;
                progress = 100.0 * numCapsComputed / numCapsToCompute;
                if (progress > progressLimiter)
                {
                    progressLimiter = progress + 0.1;
                    printf("progress: %3.2f%%\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b", progress);
                    fflush(stdout);
                }
            }
        }
    }

    printf("done                 \n");

    // start displaying spheres
    ros::Rate r(1.0);

    ros::NodeHandle n;
    ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("reachability_marker_array", 1, true);

    while (ros::ok())
    {
        unsigned int count = 0;
        visualization_msgs::MarkerArray markerArray;

        for(size_t i = 0; i < spheres.size(); ++i)
        {
            visualization_msgs::Marker marker;

            marker.header.frame_id = "torso_lift_link";
            marker.header.stamp = ros::Time(0);

            marker.ns = "reachability_marker";
            marker.id = count++;

            marker.action = visualization_msgs::Marker::ADD;
            marker.lifetime = ros::Duration();

            capability_map_generator::Vector position = spheres[i].first;

            marker.type = visualization_msgs::Marker::SPHERE;

            marker.pose.position.x = position.x;
            marker.pose.position.y = position.y;
            marker.pose.position.z = position.z;

            marker.scale.x = resolution - resolution/10.0;
            marker.scale.y = resolution - resolution/10.0;
            marker.scale.z = resolution - resolution/10.0;

            marker.color.r = 0.0;
            marker.color.g = 0.3;
            marker.color.b = 1.0;
            marker.color.a = 1.0;
                
            markerArray.markers.push_back(marker);

            std::vector<capability_map_generator::Vector> reachables = spheres[i].second.getReachableDirections();
            std::vector<capability_map_generator::Vector> unreachables = spheres[i].second.getUnreachableDirections();

            geometry_msgs::Point start, end;

            marker.type = visualization_msgs::Marker::LINE_LIST;

            marker.pose.position.x = 0.0;
            marker.pose.position.y = 0.0;
            marker.pose.position.z = 0.0;

            marker.scale.x = 0.0015;
            marker.scale.y = 0.0;
            marker.scale.z = 0.0;

            for (size_t j = 0; j < reachables.size(); ++j)
            {
                start.x = position.x - reachables[j].x * resolution / 2.0;
                start.y = position.y - reachables[j].y * resolution / 2.0;
                start.z = position.z - reachables[j].z * resolution / 2.0;

                end.x = position.x;
                end.y = position.y;
                end.z = position.z;

                marker.points.push_back(start);
                marker.points.push_back(end);
            }
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;

            marker.id = count++;
            markerArray.markers.push_back(marker);

            marker.points.clear();

            for (size_t j = 0; j < unreachables.size(); ++j)
            {
                start.x = position.x - unreachables[j].x * resolution / 2.0;
                start.y = position.y - unreachables[j].y * resolution / 2.0;
                start.z = position.z - unreachables[j].z * resolution / 2.0;

                end.x = position.x;
                end.y = position.y;
                end.z = position.z;

                marker.points.push_back(start);
                marker.points.push_back(end);
            }
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;

            marker.id = count++;
            markerArray.markers.push_back(marker);
        }
        // Publish the marker
        marker_pub.publish(markerArray);

        // sleep a while but listen to ros::ok()
        for (int i = 0; i < 10; ++i)
        {
            if (!ros::ok())
            {
                break;
            }
            r.sleep();
        }
    }
}





