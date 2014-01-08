#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include <tclap/CmdLine.h>

#include "capability_map/CapabilityOcTree.h"


bool getMarkerFromCapIterator(visualization_msgs::Marker* marker, const CapabilityOcTree::leaf_iterator &it)
{
    // nothing to do, return false
    if (it->getCapability().getType() == EMPTY)
    {
        return false;
    }

    bool retVal;

    double size = it.getSize();

    // phi and theta in rad for sin, cos computation
    double phiInRad = it->getCapability().getPhi() * M_PI / 180.0;
    double thetaInRad = it->getCapability().getTheta() * M_PI / 180.0;

    // for setting correct scale of capability shapes
    double tanHalfOpeningAngle;
    // be sure tan(halfOpeningAngle) doesn't get too high and does not reach infinity
    if (it->getCapability().getHalfOpeningAngle() > 89.5)
    {
        // cylinders can't have a halfOpeningAngle >= 90° (90° means it is a sphere). Cones can't be displayed with >= 90°.
        // setting halfOpeningAngle to 89.5° is accurate enough without getting too large marker shapes.
        tanHalfOpeningAngle = tan(89.5 * M_PI / 180.0);
    }
    else
    {
        tanHalfOpeningAngle = tan(it->getCapability().getHalfOpeningAngle() * M_PI / 180.0);
    }
    double scalingFactor = tanHalfOpeningAngle > 1.0 ? 1.0 / tanHalfOpeningAngle : 1.0;

    // init color to base color black
    marker->color.r = 0.0;
    marker->color.g = 0.0;
    marker->color.b = 0.0;
    marker->color.a = 1.0;

    // start and end points of arrow (used by cone)
    geometry_msgs::Point start, end;

    switch (it->getCapability().getType())
    {
        case SPHERE:
            marker->type = visualization_msgs::Marker::SPHERE;

            marker->pose.position.x = it.getX();
            marker->pose.position.y = it.getY();
            marker->pose.position.z = it.getZ();

            marker->scale.x = size;
            marker->scale.y = size;
            marker->scale.z = size;

            // sphere color is magenta
            marker->color.r = 1.0;
            marker->color.b = 1.0;

            retVal = true;
            break;

        case CONE:
            // TODO: cones with halfOpeningAngle > 90° can't be displayed (for now those are clamped to ~ 89.5°)
            marker->type = visualization_msgs::Marker::ARROW;

            start.x = it.getX() - (size / 2.0) * sin(thetaInRad) * cos(phiInRad);
            start.y = it.getY() - (size / 2.0) * sin(thetaInRad) * sin(phiInRad);
            start.z = it.getZ() - (size / 2.0) * cos(thetaInRad);

            end.x = it.getX();
            end.y = it.getY();
            end.z = it.getZ();

            marker->points.push_back(start);
            marker->points.push_back(end);

            marker->scale.x = 0.0;
            marker->scale.y = tanHalfOpeningAngle * size * scalingFactor;
            marker->scale.z = size * scalingFactor / 2.0;

            // cone color is blue
            marker->color.b = 1.0;

            retVal = true;
            break;

        case CYLINDER_1:
            // as marker type for cylinder_1 a arrow without head is used, which is faster than computing a quaternion
            marker->type = visualization_msgs::Marker::ARROW;

            start.x = it.getX() - (size * scalingFactor / 2.0) * sin(thetaInRad) * cos(phiInRad);
            start.y = it.getY() - (size * scalingFactor / 2.0) * sin(thetaInRad) * sin(phiInRad);
            start.z = it.getZ() - (size * scalingFactor / 2.0) * cos(thetaInRad);

            end.x = it.getX() + (size * scalingFactor / 2.0) * sin(thetaInRad) * cos(phiInRad);
            end.y = it.getY() + (size * scalingFactor / 2.0) * sin(thetaInRad) * sin(phiInRad);
            end.z = it.getZ() + (size * scalingFactor / 2.0) * cos(thetaInRad);

            marker->points.push_back(start);
            marker->points.push_back(end);

            marker->scale.x = tanHalfOpeningAngle * size * scalingFactor;
            marker->scale.y = 0.0;
            marker->scale.z = 0.001;

            // cylinder_1 color is red
            marker->color.r = 1.0;

            retVal = true;
            break;

        case CYLINDER_2:
            // as marker type for cylinder_2 a arrow without head is used, which is faster than computing a quaternion
            marker->type = visualization_msgs::Marker::ARROW;

            start.x = it.getX() - (tanHalfOpeningAngle * size * scalingFactor / 2.0) * sin(thetaInRad) * cos(phiInRad);
            start.y = it.getY() - (tanHalfOpeningAngle * size * scalingFactor / 2.0) * sin(thetaInRad) * sin(phiInRad);
            start.z = it.getZ() - (tanHalfOpeningAngle * size * scalingFactor / 2.0) * cos(thetaInRad);

            end.x = it.getX() + (tanHalfOpeningAngle * size * scalingFactor / 2.0) * sin(thetaInRad) * cos(phiInRad);
            end.y = it.getY() + (tanHalfOpeningAngle * size * scalingFactor / 2.0) * sin(thetaInRad) * sin(phiInRad);
            end.z = it.getZ() + (tanHalfOpeningAngle * size * scalingFactor / 2.0) * cos(thetaInRad);

            marker->points.push_back(start);
            marker->points.push_back(end);

            marker->scale.x = size * scalingFactor;
            marker->scale.y = 0.0;
            marker->scale.z = 0.001;

            // cylinder_2 color is green
            marker->color.g = 1.0;

            retVal = true;
            break;

        default:
            retVal = false;
    }

    // TODO: set color according to shape fit error (later)
    marker->color.r = it->getCapability().getShapeFitError() / 100.0;
    marker->color.g = 0.0;
    marker->color.b = 1.0 - it->getCapability().getShapeFitError() / 100.0;

    return retVal;
}




int main(int argc, char** argv )
{
    // TODO: get capability map file from arg

    ros::init(argc, argv, "capability_shapes");

    // arguments
    TCLAP::CmdLine cmd("Visualizes the capability map given by argument", ' ', "1.0");

    TCLAP::ValueArg<std::string> pathNameArg("p", "path", "path and filename of the capability map to be visualized.\n\
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

    CapabilityOcTree* visTree = dynamic_cast<CapabilityOcTree*>(AbstractOcTree::read(pathName));

    /*if (!visTree.read(pathName))
    {
        ROS_ERROR("Error: Couldn't load file %s", pathName.c_str());
    }*/

    ros::NodeHandle n;
    ros::Rate r(1);

    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("capability_marker", 1);

    // TODO: remove tree when done debugging
    CapabilityOcTree tree(0.1);

    Capability emptyCap;
    Capability cap1(SPHERE, 0.0, 0.0, 0.0);
    Capability cap2(CONE, 0.0, 90.0, 10.0);
    Capability cap3(CONE, 90.0, 0.0, 45.0);
    Capability cap4(CONE, 40.0, 20.0, 100.0);
    Capability cap5(CONE, 180.0, 120.0, 70.0);
    Capability cap6(CYLINDER_1, 0.0, 0.0, 10.0);
    Capability cap7(CYLINDER_1, 0.0, 0.0, 90.0);
    Capability cap8(CYLINDER_1, 40.0, 50.0, 45.0);
    Capability cap9(CYLINDER_2, 0.0, 0.0, 10.0);
    Capability cap10(CYLINDER_2, 0.0, 0.0, 45.0);
    Capability cap11(CYLINDER_2, 90.0, 90.0, 89.0);

    tree.setNodeCapability(0.0, 0.0, 1.0, emptyCap);
    tree.setNodeCapability(0.2, 0.0, 1.0, cap1);
    tree.setNodeCapability(0.4, 0.0, 1.0, cap2);
    tree.setNodeCapability(0.6, 0.0, 1.0, cap3);
    tree.setNodeCapability(0.8, 0.0, 1.0, cap4);
    tree.setNodeCapability(1.0, 0.0, 1.0, cap5);
    tree.setNodeCapability(1.2, 0.0, 1.0, cap6);
    tree.setNodeCapability(1.4, 0.0, 1.0, cap7);
    tree.setNodeCapability(1.6, 0.0, 1.0, cap8);
    tree.setNodeCapability(1.8, 0.0, 1.0, cap9);
    tree.setNodeCapability(2.0, 0.0, 1.0, cap10);
    tree.setNodeCapability(2.2, 0.0, 1.0, cap11);


    while (ros::ok())
    {
        unsigned int count = 0;
        for(CapabilityOcTree::leaf_iterator it = tree.begin_leafs(), end = tree.end_leafs(); it != end; ++it)
        {
            visualization_msgs::Marker marker;

            marker.header.frame_id = "/map";
            marker.header.stamp = ros::Time::now();

            marker.ns = "capability_shapes";
            marker.id = count++;

            marker.action = visualization_msgs::Marker::ADD;
            marker.lifetime = ros::Duration();

            if (!getMarkerFromCapIterator(&marker, it))
            {
                continue;
            }
            // Publish the marker
            marker_pub.publish(marker);

            // draw a cube around the capability marker to see the volume
            visualization_msgs::Marker cubeMarker;

            cubeMarker.header.frame_id = "/map";
            cubeMarker.header.stamp = ros::Time::now();

            cubeMarker.ns = "capability_shapes";
            cubeMarker.id = count++;

            cubeMarker.action = visualization_msgs::Marker::ADD;
            cubeMarker.lifetime = ros::Duration();

            cubeMarker.type = visualization_msgs::Marker::CUBE;

            cubeMarker.color.r = 1.0;
            cubeMarker.color.g = 1.0;
            cubeMarker.color.b = 1.0;
            cubeMarker.color.a = 0.2;

            cubeMarker.pose.position.x = it.getX();
            cubeMarker.pose.position.y = it.getY();
            cubeMarker.pose.position.z = it.getZ();

            cubeMarker.scale.x = it.getSize();
            cubeMarker.scale.y = it.getSize();
            cubeMarker.scale.z = it.getSize();

            // Publish the marker
            marker_pub.publish(cubeMarker);
        }
        for(CapabilityOcTree::leaf_iterator it = visTree->begin_leafs(), end = visTree->end_leafs(); it != end; ++it)
        {
            visualization_msgs::Marker marker;

            marker.header.frame_id = "/map";
            marker.header.stamp = ros::Time::now();

            marker.ns = "capability_shapes";
            marker.id = count++;

            marker.action = visualization_msgs::Marker::ADD;
            marker.lifetime = ros::Duration();

            if (!getMarkerFromCapIterator(&marker, it))
            {
                continue;
            }
            // Publish the marker
            marker_pub.publish(marker);
        }

      r.sleep();
    }
}

