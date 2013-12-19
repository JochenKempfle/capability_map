#ifndef REACHABILITY_DUMMY_INTERFACE_H
#define REACHABILITY_DUMMY_INTERFACE_H

#include "capability_map_generator/ReachabilityInterface.h"

namespace capability_map_generator
{

    class ReachabilityDummyInterface : public ReachabilityInterface
    {
        public:
            bool isReachable(const octomap::pose6d & pose) const;

    };

};

#endif
