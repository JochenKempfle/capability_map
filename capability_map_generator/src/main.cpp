#include "capability_map_generator/ReachabilityInterface.h"
#include "capability_map_generator/Vector.h"
#include "capability_map_generator/ReachabilitySphere.h"
#include "capability_map/CapabilityOcTree.h"
#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <tclap/CmdLine.h>
#include <sys/stat.h>
#include <string>
#include <vector>
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

// creates the given directory chain
bool makeDir(const std::string &path, const int filemode)
{
    if(mkdir(path.c_str(), filemode) && errno != EEXIST)
    {
        size_t pos = path.find_last_of("/");
        // check if path is root, if so return false, directory chain could not be created
        if (pos == 0 || pos == std::string::npos)
        {
            return false;
        }
        std::string parentPath = path.substr(0, pos);
        // recursively create directory chain of parent paths
        if (!makeDir(parentPath, filemode))
        {
            return false;
        }
        // parent directory should now exist
        if (mkdir(path.c_str(), filemode))
        {
            return false;
        }
    }
    return true;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "capability_map_generator");

    ros::NodeHandle nhPriv("~");
    
    // load the reachability interface given by launch file
    boost::shared_ptr<capability_map_generator::ReachabilityInterface> ri = loadReachabilityInterface(nhPriv);

    ROS_ASSERT(ri);

    std::string msg = "Generates a capability map of the region specified by given bounding box.";
    TCLAP::CmdLine cmd(msg, ' ', "1.0");

    msg = "Number of samples per voxel. Default is 200.";
    TCLAP::ValueArg<unsigned int> numSamplesArg("n", "numSamples", msg, false, 200, "integer");

    msg = "Distance between two voxels in meter, default is 0.1 m.";
    TCLAP::ValueArg<double> resolutionArg("r", "resolution", msg, false, 0.1, "floating point");

    msg = "filename and path where the capability map will be created.\nExample: -p mydir/mysubdir/filename.cpm";
    TCLAP::ValueArg<std::string> pathNameArg("p", "path", msg, true, "./capability_map.cpm", "string");

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

    // msg = "If set, writes a log file containing time required and number of computed capabilities to map_name.cpm.log";
    // TCLAP::SwitchArg logArg("l", "log", msg, false);

    cmd.add(zArg);
    cmd.add(yArg);
    cmd.add(xArg);
    // cmd.add(logArg);
    cmd.add(resolutionArg);
    cmd.add(numSamplesArg);
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
    
    // get values from arguments
    int numSamples = numSamplesArg.getValue();
    double resolution = resolutionArg.getValue();
    std::string pathName = pathNameArg.getValue();
    // bool loggingEnabled = logArg.getValue();


    if (pathName.find_first_of('/') != 0)
    {
        ROS_ERROR("Error: Path to %s is not absolute! The capability_map_generator node may not \
                   run in your current working directory, so only absolute paths are accepted!\
                   \nExample: /home/user/capability_maps/map1.cpm", pathName.c_str());
        ros::shutdown();
        exit(1);
    }
    if (pathName.find_last_of('/') == pathName.length())
    {
        ROS_ERROR("Error: Path to %s does not name a valid file!\
                   \nExample: /home/user/capability_maps/map1.cpm", pathName.c_str());
        ros::shutdown();
        exit(1);
    }
    unsigned int pos = pathName.find_last_of('/');
    std::string path = pathName.substr(0, pos);

    
    if (!makeDir(path, 0775))
    {
        ROS_ERROR("Error: Could not create directory to %s", path.c_str());
        ros::shutdown();
        exit(1);
    }

    std::string fileName = pathName.substr(pos + 1);
    pos = fileName.find_last_of('.');

    // append a ".cpm" to file name if not already there
    if (pos == std::string::npos || fileName.substr(pos) != ".cpm")
    {
        pathName += ".cpm";
    }

    // check if values are valid
    if (numSamples <= 0)
    {
        ROS_ERROR("Error: number of samples must be positive and greater than 0");
        ros::shutdown();
        exit(1);
    }
    if (resolution <= 0.0)
    {
        ROS_ERROR("Error: resolution must be positive and greater than 0.0");
        ros::shutdown();
        exit(1);
    }
    
    // create a CapabilityOcTree with resolution given by argument
    CapabilityOcTree tree(resolution);

    std::string base_name;
    if(!nhPriv.getParam("base_name", base_name))
    {
        ROS_ERROR("No base_name defined!");
        ros::shutdown();
        exit(1);
    }
    std::string tip_name;
    if(!nhPriv.getParam("tip_name", tip_name))
    {
        ROS_ERROR("No tip_name defined!");
        ros::shutdown();
        exit(1);
    }

    tree.setBaseName(base_name);
    tree.setTipName(tip_name);

    
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
    // capability_map_generator::ReachabilityInterface::BoundingBox bbx = ri->getBoundingBox();

    /*
    double startX = bbx.getStartPoint().x < bbx.getEndPoint().x ? bbx.getStartPoint().x : bbx.getEndPoint().x;
    double endX = bbx.getStartPoint().x < bbx.getEndPoint().x ? bbx.getEndPoint().x : bbx.getStartPoint().x;
    double startY = bbx.getStartPoint().y < bbx.getEndPoint().y ? bbx.getStartPoint().y : bbx.getEndPoint().y;
    double endY = bbx.getStartPoint().y < bbx.getEndPoint().y ? bbx.getEndPoint().y : bbx.getStartPoint().y;
    double startZ = bbx.getStartPoint().z < bbx.getEndPoint().z ? bbx.getStartPoint().z : bbx.getEndPoint().z;
    double endZ = bbx.getStartPoint().z < bbx.getEndPoint().z ? bbx.getEndPoint().z : bbx.getStartPoint().z;
    */

    // get x, y and z values and sort them
    std::vector<double> xValues = xArg.getValue();
    std::vector<double> yValues = yArg.getValue();
    std::vector<double> zValues = zArg.getValue();

    std::sort(xValues.begin(), xValues.end());
    std::sort(yValues.begin(), yValues.end());
    std::sort(zValues.begin(), zValues.end());

    // get and adjust the boundaries for iteration
    double startX = tree.getAlignment(xValues[0]);
    double endX = tree.getAlignment(xValues[xValues.size() - 1]);
    double startY = tree.getAlignment(yValues[0]);
    double endY = tree.getAlignment(yValues[yValues.size() - 1]);
    double startZ = tree.getAlignment(zValues[0]);
    double endZ = tree.getAlignment(zValues[zValues.size() - 1]);

    // wether the computation was aborted or not and at which position
    bool aborted = false;
    double abortPosX, abortPosY, abortPosZ;

    double numCapsToCompute = ((endX - startX) / resolution + 1.0) * ((endY - startY) / resolution + 1.0)
                              * ((endZ - startZ) / resolution + 1.0);
    double numCapsComputed = 0.0;

    ROS_INFO("Number of capabilities to compute: %d", (unsigned int)numCapsToCompute);

    //  add a small value to end due to floating point precision
    endX += resolution/100.0;
    endY += resolution/100.0;
    endZ += resolution/100.0;

    // progress in percent
    double progress = 0.0;
    double progressLimiter = 0.0;

    ros::Time startTime = ros::Time::now();

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
                tree.setNodeCapability(x, y, z, sphere.convertToCapability());
                sphere.clear();

                numCapsComputed += 1.0;
                progress = 100.0 * numCapsComputed / numCapsToCompute;
                if (progress > progressLimiter)
                {
                    progressLimiter = progress + 0.1;
                    printf("progress: %3.2f%%\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b", progress);
                    fflush(stdout);
                }

                if (!ros::ok())
                {
                    // the computation was aborted, remember actual x, y and z values
                    aborted = true;
                    abortPosX = x;
                    abortPosY = y;
                    abortPosZ = z;
                    break;
                }
            }
            if (aborted)
            {
                break;
            }
        }
        if (aborted)
        {
            break;
        }
    }

    ros::Time endTime = ros::Time::now();

    if (aborted)
    {
        printf("Aborted at x: %g, y: %g, z: %g\n", abortPosX, abortPosY, abortPosZ);
        unsigned int numCapsComputed = ((abortPosX - startX) / resolution + 1.0) * ((endY - startY) / resolution + 1.0)
                                 * ((endZ - startZ) / resolution + 1.0) - ((abortPosY - startY) / resolution + 1.0)
                                 * ((abortPosZ - startZ) / resolution + 1.0) - ((abortPosZ - startZ) / resolution + 1.0);
        printf("Successfully computed %d of %d capabilities, %d capabilities are left.\n", numCapsComputed, (unsigned int)numCapsToCompute,
                                                                                           (unsigned int)numCapsToCompute - numCapsComputed);
    }
    else
    {
        printf("done              \n");
        printf("Successfully computed %d capabilities.\n", (unsigned int)numCapsToCompute);
    }

    int secs = int((endTime - startTime).toSec());
    int hours = secs / 3600;
    secs %= 3600;
    int mins = secs / 60;
    secs %= 60;
    printf("Time passed: %d h %d min %d sec\n", hours, mins, secs);

    if (!tree.writeFile(pathName))
    {
        ROS_ERROR("Error: Could not write to file %s\n", pathName.c_str());
        ros::shutdown();
        exit(1);
    }
    else
    {
        ROS_INFO("Capability map written to file %s", pathName.c_str());
    }

    if (aborted)
    {
        printf("\nThe already computed part of the capability map doesn't need to be recalculated.");
        printf("Simply restart the program with following arguments:\n");
        if (abortPosX != endX)
        {
            printf("-p %s -n %d -r %g -x %g -x %g -y %g -y %g -z %g -z %g\n\n", (pathName + "_2").c_str(),
                                      numSamples, resolution, abortPosX, endX, startY, endY, startZ, endZ);
        }
        else if (abortPosY != endY)
        {
            printf("-p %s -n %d -r %g -x %g -x %g -y %g -y %g -z %g -z %g\n\n", (pathName + "_2").c_str(),
                                      numSamples, resolution, abortPosX, endX, abortPosY, endY, startZ, endZ);
        }
        else
        {
            printf("-p %s -n %d -r %g -x %g -x %g -y %g -y %g -z %g -z %g\n\n", (pathName + "_2").c_str(),
                                      numSamples, resolution, abortPosX, endX, abortPosY, endY, abortPosZ, endZ);
        }
        printf("When finished start merge_capability_maps as follows:\n");
        printf("rosrun capability_map_generator merge_capability_maps -i %s -i %s -o %s\n\n", pathName.c_str(),
                                                               (pathName + "_2").c_str(), pathName.c_str());
    }
}





