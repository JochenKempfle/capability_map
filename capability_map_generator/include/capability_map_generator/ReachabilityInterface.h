#ifndef REACHABILITY_INTERFACE_H
#define REACHABILITY_INTERFACE_H

#include <octomap/octomap.h>


namespace capability_map_generator
{

class ReachabilityInterface
{
  public:
    ReachabilityInterface();

    virtual bool isReachable(const octomath::Pose6D &pose) const = 0;
};

}

#endif // REACHABILITY_INTERFACE_H

