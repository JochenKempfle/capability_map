#include "capability_map_generator/ReachabilityInterface.h"
#include "capability_map_generator/Vector.h"
#include "capability_map_generator/ReachabilitySphere.h"
#include "capability_map/CapabilityOcTree.h"
#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <tclap/CmdLine.h>
#include <string>


static pluginlib::ClassLoader<capability_map_generator::ReachabilityInterface>* s_ReachbilityInterface = NULL;

boost::shared_ptr<capability_map_generator::ReachabilityInterface> loadReachabilityInterface(ros::NodeHandle & nh)
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

    TCLAP::ValueArg<std::string> pathNameArg("p", "path", "filename and path where the capability map will be created.\n\
                                             Example: -p mydir/mysubdir/filename.cpm", true, "./capability_map.cpm", "string");

    cmd.add(numSamplesArg);
    cmd.add(resolutionArg);
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
    
    // get values from arguments
    int numSamples = numSamplesArg.getValue();
    double resolution = resolutionArg.getValue();
    std::string pathName = pathNameArg.getValue();
    
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
    
    // create a CapabilityOcTree with resolution given by argument
    CapabilityOcTree tree(resolution);
    
    // get coordinates of equally distributed points over a sphere
    std::vector<capability_map_generator::Vector> spherePoints = distributePointsOnSphere(numSamples);
    
    // calculate the corresponding quaternions for the sphere point coordinates
    std::vector<octomath::Quaternion> quaternions(spherePoints.size());
    for (size_t i = 0; i < spherePoints.size(); ++i)
    {
        quaternions[i] = vectorToQuaternion(spherePoints[i]);
    }

    // the reachability sphere which finally creates a capability
    capability_map_generator::ReachabilitySphere sphere;

    // get and adjust the boundaries for iteration
    capability_map_generator::ReachabilityInterface::BoundingBox bbx = ri->getBoundingBox();
    bbx.getStartPoint();

    for(double x = bbx.getStartPoint().x; x > bbx.getEndPoint().x; x -= resolution)
    {
        for(double y = bbx.getStartPoint().y; y > bbx.getEndPoint().y; y -= resolution)
        {
            for(double z = bbx.getStartPoint().z; z > bbx.getEndPoint().z; z -= resolution)
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
                tree.setNodeCapability(x, y, z, sphere.convertToCapability());
                sphere.clear();
            }
        }
    }
    if (!tree.write(pathName))
    {
        ROS_ERROR("Error: could not write to file %s", pathName.c_str());
        exit(1);
    }
}





