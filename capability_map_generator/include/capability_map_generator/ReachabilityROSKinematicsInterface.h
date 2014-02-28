#ifndef REACHABILITY_ROS_KINEMATICS_H
#define REACHABILITY_ROS_KINEMATICS_H

#include <ros/ros.h>
#include "capability_map_generator/ReachabilityInterface.h"
#include "kinematics_base/kinematics_base.h"

namespace capability_map_generator
{

class ReachabilityROSKinematicsInterface : public ReachabilityInterface
{
  public:
    ReachabilityROSKinematicsInterface();

    bool isReachable(const octomath::Pose6D &pose) const;

  private:
    boost::shared_ptr<kinematics::KinematicsBase> kinematics;

    std::vector<double> seed;
};

}

#endif
