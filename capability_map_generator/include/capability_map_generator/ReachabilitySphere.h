#include <vector>
#include <utility>
#include <gtest/gtest.h>

#include "capability_map/CapabilityOcTreeNode.h"


#ifndef REACHABILITYSPHERE_H
#define REACHABILITYSPHERE_H


class ReachabilitySphere
{
  public:
    ReachabilitySphere();
    ~ReachabilitySphere();

    FRIEND_TEST(ReachabilitySphere, appendDirection);

    // appends a specific direction in sphere coordinates and determines if this direction is reachable or not
    void appendDirection(double phi, double theta, bool reachable);
    // appends a specific direction in cartesian coordinates pointing from origin to a point in this direction
    // and determines if this direction is reachable or not
    void appendDirection(double x, double y, double z, bool reachable);

    Capability convertToCapability();

    // helper class used for coordinates and vectors
    class Vector
    {
      public:
	Vector() { }
	~Vector() { }
        Vector(double xVal, double yVal, double zVal) : x(xVal), y(yVal), z(zVal) { }
        Vector operator*(double rhs) const { return Vector(x * rhs, y * rhs, z * rhs); }
        double x, y, z;
    };

  protected:

    FRIEND_TEST(ReachabilitySphere, getPrincipalComponents);

    // returns principal components (eigenvectors) ordered from smallest to greatest eigenvalue
    std::vector<Vector> getPrincipalComponents(const std::vector<Vector> &coords) const;

    FRIEND_TEST(ReachabilitySphere, fitCone);
    FRIEND_TEST(ReachabilitySphere, fitCylinder_1);
    FRIEND_TEST(ReachabilitySphere, fitCylinder_2);


    // functions that try to fit a certain shape to the reachable directions, return type is pair(angle, shapeFitError)
    std::pair<double, double> fitCone(const Vector &axis) const;
    std::pair<double, double> fitCylinder_1(const Vector &axis) const;
    std::pair<double, double> fitCylinder_2(const Vector &axis) const;

  private:

    std::vector<Vector> _reachableDirections;
    std::vector<Vector> _unreachableDirections;
};

#endif // REACHABILITYSPHERE_H

