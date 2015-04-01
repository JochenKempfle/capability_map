// Copyright (c) 2014, Jochen Kempfle
// All rights reserved.

/*
Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/


#include <vector>
#include <utility>
#include <gtest/gtest.h>

#include "capability_map/CapabilityOcTreeNode.h"
#include "capability_map_generator/Vector.h"


#ifndef REACHABILITYSPHERE_H
#define REACHABILITYSPHERE_H

namespace capability_map_generator
{

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

    // removes all appended directions
    void clear();

    // get reachable directions
    std::vector<Vector> getReachableDirections() const { return _reachableDirections; }

    // get unreachable directions
    std::vector<Vector> getUnreachableDirections() const { return _unreachableDirections; }

    Capability convertToCapability();


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

}

#endif // REACHABILITYSPHERE_H

