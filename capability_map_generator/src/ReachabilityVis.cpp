#include "capability_map_generator/ReachabilityInterface.h"
#include "capability_map_generator/Vector.h"
#include "capability_map_generator/ReachabilitySphere.h"
#include <visualization_msgs/MarkerArray.h>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <tclap/CmdLine.h>
#include <string>
#include <vector>
#include <utility>


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
    
    TCLAP::CmdLine cmd("Generates a capability map", ' ', "1.0");

    TCLAP::ValueArg<unsigned int> numSamplesArg("n", "numSamples", "Number of samples per voxel. Default is 200.", false, 200, "integer");

    TCLAP::ValueArg<double> resolutionArg("r", "resolution", "Distance between two voxels in meter, default is 0.1 m.", false, 0.1, "floating point");

    TCLAP::MultiArg<double> xArg("x", "x-pos", "The start/end point of the bounding box in x-direction\n\
                                             Example: -x 0.1 -x 2.3", true, "floating point");

    TCLAP::MultiArg<double> yArg("y", "y-pos", "The start/end point of the bounding box in y-direction\n\
                                             Example: -y 0.1 -y 2.3", true, "floating point");

    TCLAP::MultiArg<double> zArg("z", "z-pos", "The start/end point of the bounding box in z-direction\n\
                                             Example: -z 0.1 -z 2.3", true, "floating point");

    cmd.add(numSamplesArg);
    cmd.add(resolutionArg);
    cmd.add(xArg);
    cmd.add(yArg);
    cmd.add(zArg);

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
        // TODO: maybe set numSamples to default and inform the user
        ROS_ERROR("Error: number of samples must be positive and greater than 0");
        exit(1);
    }
    if (resolution <= 0.0)
    {
        // TODO: maybe set numSamples to default and inform the user
        ROS_ERROR("Error: resolution must be positive and greater than 0.0");
        exit(1);
    }

    if (xArg.getValue().size() != 2)
    {
        ROS_ERROR("Error: Exactly 2 values for x-position must be given as argument");
        ros::shutdown();
        exit(1);
    }
    else if (xArg.getValue().size() != 2)
    {
        ROS_ERROR("Error: Exactly 2 values for y-position must be given as argument");
        ros::shutdown();
        exit(1);
    }
    else if (xArg.getValue().size() != 2)
    {
        ROS_ERROR("Error: Exactly 2 values for z-position must be given as argument");
        ros::shutdown();
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

    // get and adjust the boundaries for iteration
    double startX = xArg.getValue()[0] < xArg.getValue()[1] ? xArg.getValue()[0] : xArg.getValue()[1];
    double endX = xArg.getValue()[0] < xArg.getValue()[1] ? xArg.getValue()[1] : xArg.getValue()[0];
    double startY = yArg.getValue()[0] < yArg.getValue()[1] ? yArg.getValue()[0] : yArg.getValue()[1];
    double endY = yArg.getValue()[0] < yArg.getValue()[1] ? yArg.getValue()[1] : yArg.getValue()[0];
    double startZ = zArg.getValue()[0] < zArg.getValue()[1] ? zArg.getValue()[0] : zArg.getValue()[1];
    double endZ = zArg.getValue()[0] < zArg.getValue()[1] ? zArg.getValue()[1] : zArg.getValue()[0];

    // progress in percent
    double progress = 0.0;
    double updateProgress = 100.0 / (((endX - startX) / resolution) * ((endY - startY) / resolution));

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
            }
            progress += updateProgress;
            printf("progress: %3.1f%%\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b", progress);
            fflush(stdout);
        }
    }

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





