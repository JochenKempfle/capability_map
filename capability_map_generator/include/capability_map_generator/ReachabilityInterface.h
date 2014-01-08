#ifndef REACHABILITY_INTERFACE_H
#define REACHABILITY_INTERFACE_H

#include <octomap/octomap.h>
#include "capability_map_generator/Vector.h"


namespace capability_map_generator
{

class ReachabilityInterface
{
  public:
    ReachabilityInterface();

    class BoundingBox
    {
      public:
        BoundingBox(const Vector &startPoint, const Vector &endPoint) : _startPoint(startPoint), _endPoint(endPoint) { }

        void setStartPoint(const Vector &startPoint) { _startPoint = startPoint; }
        void setEndPoint(const Vector &endPoint) { _endPoint = endPoint; }
        inline Vector getStartPoint() const { return _startPoint; }
        inline Vector getEndPoint() const { return _endPoint; }

      private:
        Vector _startPoint;
        Vector _endPoint;
    };

    virtual bool isReachable(const octomath::Pose6D &pose) const = 0;

    virtual BoundingBox getBoundingBox() const = 0;
};

}

#endif // REACHABILITY_INTERFACE_H

