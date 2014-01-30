#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <cmath>
#include <tclap/CmdLine.h>
#include <string>
#include <vector>

#include "capability_map/CapabilityOcTree.h"


void setMarkerColor(visualization_msgs::Marker* marker, double value)
{
    if (value < 0.0)
    {
        value = 0.0;
    }
    else if (value > 1.0)
    {
        value = 1.0;
    }

    if (value <= 0.25)
    {
        marker->color.r = 0.0;
        marker->color.g = value * 4.0;
        marker->color.b = 1.0;
    }
    else if (value <= 0.5)
    {
        marker->color.r = 0.0;
        marker->color.g = 1.0;
        marker->color.b = 1.0 - (value - 0.25) * 4.0 ;
    }
    else if (value <= 0.75)
    {
        marker->color.r = (value - 0.5) * 4.0;
        marker->color.g = 1.0;
        marker->color.b = 0.0;
    }
    else
    {
        marker->color.r = 1.0;
        marker->color.g = 1.0 - (value - 0.75) * 4.0;
        marker->color.b = 0.0;
    }
    marker->color.a = 1.0;
}


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

    double halfOpeningAngle = it->getCapability().getHalfOpeningAngle();
    
    // cone needs special treatment if halfOpeningAngle is greater than 90.0°
    if (it->getCapability().getType() == CONE && it->getCapability().getHalfOpeningAngle() > 90.0)
    {
        phiInRad = M_PI + phiInRad;
        if (phiInRad > 2.0 * M_PI)
        {
            phiInRad -= 2.0 * M_PI;
        }
        thetaInRad = M_PI - thetaInRad;
        halfOpeningAngle = 180.0 - halfOpeningAngle;
    }
    
    // for setting correct scale of capability shapes
    double tanHalfOpeningAngle;
    
    // be sure tan(halfOpeningAngle) doesn't get too high and does not reach infinity
    if (halfOpeningAngle > 89.5)
    {
        // cylinders can't have a halfOpeningAngle >= 90° (90° means it is a sphere). Cones can't be displayed with >= 90°.
        // setting halfOpeningAngle to 89.5° is accurate enough without getting too large marker shapes.
        tanHalfOpeningAngle = tan(89.5 * M_PI / 180.0);
    }
    else
    {
        tanHalfOpeningAngle = tan(halfOpeningAngle * M_PI / 180.0);
    }
    double scalingFactor = tanHalfOpeningAngle > 1.0 ? 1.0 / tanHalfOpeningAngle : 1.0;

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

            retVal = true;
            break;

        case CONE:
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

            retVal = true;
            break;

        default:
            retVal = false;
    }

    return retVal;
}




int main(int argc, char** argv )
{
    ros::init(argc, argv, "capability_shapes");

    // arguments
    TCLAP::CmdLine cmd("Visualizes the capability map given by argument", ' ', "1.0");

    TCLAP::ValueArg<std::string> pathNameArg("p", "path", "Path and filename of the capability map to be visualized.\n\
                                             Example: -p mydir/mysubdir/filename.cpm", true, "./capability_map.cpm", "string");

    std::string msg;
    msg = "Specifies the region in x-direction to be visualized.\n\
           If no x-value is given, depending on y- and z-values, all stored capabilities in x-direction are displayed.\n\
           If only one x-value is given, a slice (or a point) at this position depending on y- and z-values gets computed.\n\
           If more than 2 values are given, the boundaries are from min(x1, x2, ...) to max(x1, x2, ...).\n\
           Example: -x -0.1 -x 2.3";
    TCLAP::MultiArg<double> xArg("x", "x-pos", msg, false, "floating point");

    msg = "Specifies the region in y-direction to be visualized.\n\
           If no y-value is given, depending on x- and z-values, all stored capabilities in y-direction are displayed.\n\
           If only one y-value is given, a slice (or a point) at this position depending on x- and z-values gets computed.\n\
           If more than 2 values are given, the boundaries are from min(y1, y2, ...) to max(y1, y2, ...).\n\
           Example: -y -0.1 -y 2.3";
    TCLAP::MultiArg<double> yArg("y", "y-pos", msg, false, "floating point");

    msg = "Specifies the region in z-direction to be visualized.\n\
           If no z-value is given, depending on x- and y-values, all stored capabilities in z-direction are displayed.\n\
           If only one z-value is given, a slice (or a point) at this position depending on x- and y-values gets computed.\n\
           If more than 2 values are given, the boundaries are from min(z1, z2, ...) to max(z1, z2, ...).\n\
           Example: -z -0.1 -z 2.3";
    TCLAP::MultiArg<double> zArg("z", "z-pos", msg, false, "floating point");

    msg = "If set, shows the capability shapes in a color depending on the surface area which is reachable";
    TCLAP::SwitchArg areaArg("a", "area", msg, false);

    msg = "If set, shows a color table from blue = best to red = worst";
    TCLAP::SwitchArg colorArg("c", "colortable", msg, false);

    cmd.add(zArg);
    cmd.add(yArg);
    cmd.add(xArg);
    cmd.add(colorArg);
    cmd.add(areaArg);
    cmd.add(pathNameArg);

    // parse arguments with TCLAP
    try
    {
        cmd.parse(argc, argv);
    }
    catch (TCLAP::ArgException &e)  // catch any exceptions
    {
        ROS_ERROR("Error: %s for argument %s", e.error().c_str(), e.argId().c_str());
        ros::shutdown();
        exit(1);
    }

    bool showAreaColor = areaArg.getValue();
    bool showColorTable = colorArg.getValue();

    std::string pathName = pathNameArg.getValue();
    CapabilityOcTree* tree = CapabilityOcTree::readFile(pathName);

    if (tree == NULL)
    {
        ROS_ERROR("Error: Capability map could not be loaded.\n");
        ros::shutdown();
        exit(1);
    }

    std::string frame = tree->getBaseName();

    ROS_INFO("Base frame is: %s.", frame.c_str());
    ROS_INFO("Tip frame is: %s.", tree->getTipName().c_str());
    ROS_INFO("Resolution is: %g\n", tree->getResolution());

    // get x, y and z values and sort them
    std::vector<double> xValues = xArg.getValue();
    std::vector<double> yValues = yArg.getValue();
    std::vector<double> zValues = zArg.getValue();

    std::sort(xValues.begin(), xValues.end());
    std::sort(yValues.begin(), yValues.end());
    std::sort(zValues.begin(), zValues.end());

    bool xIsSet = xValues.size() > 0 ? true : false;
    bool yIsSet = yValues.size() > 0 ? true : false;
    bool zIsSet = zValues.size() > 0 ? true : false;

    double startX = 0.0, endX = 0.0, startY = 0.0, endY = 0.0, startZ = 0.0, endZ = 0.0;

    // get and adjust the boundaries for iteration (add a small value to end due to floating point precision)
    if (xIsSet)
    {
        startX = tree->getAlignment(xValues[0]);
        endX = tree->getAlignment(xValues[xValues.size() - 1]) + tree->getResolution()/100.0;
    }
    if (yIsSet)
    {
        startY = tree->getAlignment(yValues[0]);
        endY = tree->getAlignment(yValues[yValues.size() - 1]) + tree->getResolution()/100.0;
    }
    if (zIsSet)
    {
        startZ = tree->getAlignment(zValues[0]);
        endZ = tree->getAlignment(zValues[zValues.size() - 1]) + tree->getResolution()/100.0;
    }

    ros::NodeHandle n;
    ros::Rate r(1.0);

    ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("capability_marker", 1, true);

    // TODO: remove testTree when done debugging (or better create a map with example capabilities)
 /*   CapabilityOcTree testTree(0.1);

    Capability cap0(SPHERE, 0.0, 0.0, 0.0, 0.0);
    Capability cap1(SPHERE, 0.0, 0.0, 0.0, 10.0);
    Capability cap2(SPHERE, 0.0, 0.0, 0.0, 20.0);
    Capability cap3(SPHERE, 0.0, 0.0, 0.0, 30.0);
    Capability cap4(SPHERE, 0.0, 0.0, 0.0, 40.0);
    Capability cap5(SPHERE, 0.0, 0.0, 0.0, 50.0);
    Capability cap6(SPHERE, 0.0, 0.0, 0.0, 60.0);
    Capability cap7(SPHERE, 0.0, 0.0, 0.0, 70.0);
    Capability cap8(SPHERE, 0.0, 0.0, 0.0, 80.0);
    Capability cap9(SPHERE, 0.0, 0.0, 0.0, 90.0);
    Capability cap10(SPHERE, 0.0, 0.0, 0.0, 100.0);

    Capability cap0;
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

    testTree.setNodeCapability(0.0, 0.0, 1.0, cap0);
    testTree.setNodeCapability(0.2, 0.0, 1.0, cap1);
    testTree.setNodeCapability(0.4, 0.0, 1.0, cap2);
    testTree.setNodeCapability(0.6, 0.0, 1.0, cap3);
    testTree.setNodeCapability(0.8, 0.0, 1.0, cap4);
    testTree.setNodeCapability(1.0, 0.0, 1.0, cap5);
    testTree.setNodeCapability(1.2, 0.0, 1.0, cap6);
    testTree.setNodeCapability(1.4, 0.0, 1.0, cap7);
    testTree.setNodeCapability(1.6, 0.0, 1.0, cap8);
    testTree.setNodeCapability(1.8, 0.0, 1.0, cap9);
    testTree.setNodeCapability(2.0, 0.0, 1.0, cap10);
    //testTree.setNodeCapability(2.2, 0.0, 1.0, cap11);
*/
    // remember the greatest extent in y-direction to properly set the color table outside of the capabilities
    double maxY = 0.0;

    while (ros::ok())
    {
        unsigned int count = 0;
        visualization_msgs::MarkerArray markerArray;
/*        for(CapabilityOcTree::leaf_iterator it = testTree.begin_leafs(), end = testTree.end_leafs(); it != end; ++it)
        {
            visualization_msgs::Marker marker;

            marker.header.frame_id = "/torso_lift_link";
            marker.header.stamp = ros::Time::now();

            marker.ns = "capability_shapes";
            marker.id = count++;

            marker.action = visualization_msgs::Marker::ADD;
            marker.lifetime = ros::Duration();

            if (!getMarkerFromCapIterator(&marker, it))
            {
                continue;
            }
            // push the marker into array
            markerArray.markers.push_back(marker);
/*
            // draw a cube around the capability marker to see the volume
            visualization_msgs::Marker cubeMarker;

            cubeMarker.header.frame_id = "/base_link";
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
        }*/
        // loop through all capabilities
        for (CapabilityOcTree::leaf_iterator it = tree->begin_leafs(), end = tree->end_leafs(); it != end; ++it)
        {
            // if not inside boundaries, skip actual capability
            double eps = 0.000001;
            if (xIsSet && (it.getX() + eps < startX || it.getX() - eps > endX))
            {
                continue;
            }
            if (yIsSet && (it.getY() + eps < startY || it.getY() - eps > endY))
            {
                continue;
            }
            if (zIsSet && (it.getZ() + eps < startZ || it.getZ() - eps > endZ))
            {
                continue;
            }

            visualization_msgs::Marker marker;

            marker.header.frame_id = frame;
            marker.header.stamp = ros::Time(0);

            marker.ns = "capability_shapes";
            marker.id = count++;

            marker.action = visualization_msgs::Marker::ADD;
            marker.lifetime = ros::Duration();

            // skip empty capabilities
            if (!getMarkerFromCapIterator(&marker, it))
            {
                continue;
            }

            if (showAreaColor)
            {
                // set color according to reachable surface
                setMarkerColor(&marker, (1.0 - it->getCapability().getPercentReachable() / 100.0));
            }
            else
            {
                // set color according to shape fit error
                setMarkerColor(&marker, it->getCapability().getShapeFitError() / 100.0);
            }

            markerArray.markers.push_back(marker);

            if (it->getCapability().getType() == CONE && it->getCapability().getHalfOpeningAngle() > 90.0)
            {
                marker.id = count++;
                
                marker.type = visualization_msgs::Marker::SPHERE;

                marker.pose.position.x = it.getX();
                marker.pose.position.y = it.getY();
                marker.pose.position.z = it.getZ();

                marker.scale.x = it.getSize();
                marker.scale.y = it.getSize();
                marker.scale.z = it.getSize();
                
                if (showAreaColor)
                {
                    // set color according to reachable surface
                    setMarkerColor(&marker, (1.0 - it->getCapability().getPercentReachable() / 100.0));
                }
                else
                {
                    // set color according to shape fit error
                    setMarkerColor(&marker, it->getCapability().getShapeFitError() / 100.0);
                }

                marker.color.a = 0.5;
                
                markerArray.markers.push_back(marker);
            }

            // get maximal extent in y-direction
            if (maxY < it.getY())
            {
                maxY = it.getY();
            }
        }

        if (showColorTable)
        {
            for (size_t i = 0; i < 21; ++i)
            {
                visualization_msgs::Marker marker;

                marker.header.frame_id = frame;
                marker.header.stamp = ros::Time(0);

                marker.ns = "color_table";
                marker.id = count++;

                marker.action = visualization_msgs::Marker::ADD;
                marker.lifetime = ros::Duration();
                
                marker.type = visualization_msgs::Marker::CUBE;

                marker.pose.position.x = 0.05 * (double)i;
                marker.pose.position.y = maxY + 0.5;
                marker.pose.position.z = 0.0;

                marker.scale.x = 0.05;
                marker.scale.y = 0.05;
                marker.scale.z = 0.05;

                // set color according to reachable surface
                setMarkerColor(&marker, 0.05 * (double)i);

                markerArray.markers.push_back(marker);
            }
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
    delete tree;
}

