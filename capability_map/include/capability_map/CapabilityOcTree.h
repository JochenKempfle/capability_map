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


#include <gtest/gtest.h>
#include <octomap/OcTreeBase.h>
#include <fstream>
#include <string>
#include <iostream>
#include "./CapabilityOcTreeNode.h"

#ifndef CAPABILITYOCTREE_H
#define CAPABILITYOCTREE_H

using namespace octomap;

// tree definition
class CapabilityOcTree : public OcTreeBase<CapabilityOcTreeNode>
{
  public:

    FRIEND_TEST(CapabilityOcTree, constructor);

    // Default constructor, sets resolution of leafs
    CapabilityOcTree(double resolution) : OcTreeBase<CapabilityOcTreeNode>(resolution) {}

    // virtual constructor: creates a new object of same type
    // (Covariant return type requires an up-to-date compiler)
    CapabilityOcTree* create() const {return new CapabilityOcTree(resolution); }

    std::string getTreeType() const { return "CapabilityOcTree"; }

    // writes the CapabilityOcTree to file
    bool writeFile(const std::string &filename);

    // creates a new CapabilityOcTree from given file (you need to delete the created tree yourself)
    static CapabilityOcTree* readFile(const std::string &filename);

    // returns the position at which the given coordinate ends up in the tree
    inline double getAlignment(double coordinate) { return keyToCoord(coordToKey(coordinate)); }

    FRIEND_TEST(CapabilityOcTree, set_getNodeCapability);

    // set node capability at given key or coordinate. Replaces previous capability.
    CapabilityOcTreeNode* setNodeCapability(const OcTreeKey &key, const Capability &capability);

    CapabilityOcTreeNode* setNodeCapability(const double &x, const double &y, const double &z,
                                            CAPABILITY_TYPE type, double phi, double theta,
                                            double halfOpeningAngle, double shapeFitError = 0.0);

    CapabilityOcTreeNode* setNodeCapability(const double &x, const double &y,
                                            const double &z, const Capability &capability);

    // get node capability at given coordinate
    Capability getNodeCapability(const double &x, const double &y, const double &z) const;

    FRIEND_TEST(CapabilityOcTree, isPosePossible);

    // test if a certain 5 DOF pose is possible
    bool isPosePossible(const double &x, const double &y, const double &z, const double &phi, const double &theta) const;
    bool isPosePossible(const octomap::pose6d pose) const;

    // returns all positions which have at least X percent reachable directions
    std::vector<octomath::Vector3> getPositionsWithMinReachablePercent(double percent);

    void setGroupName(const std::string &name) { _groupName = name; }
    std::string getGroupName() const { return _groupName; }

    void setBaseName(const std::string &name) { _baseName = name; }
    std::string getBaseName() const { return _baseName; }

    void setTipName(const std::string &name) { _tipName = name; }
    std::string getTipName() const { return _tipName; }

  protected:

    CapabilityOcTreeNode* setNodeCapabilityRecurs(CapabilityOcTreeNode* node, bool node_just_created, const OcTreeKey& key,
                           unsigned int depth, const Capability &capability);

    /**
     * Static member object which ensures that this OcTree's prototype
     * ends up in the classIDMapping only once
     */
    class StaticMemberInitializer
    {
      public:
        StaticMemberInitializer()
        {
            CapabilityOcTree* tree = new CapabilityOcTree(0.1);
            AbstractOcTree::registerTreeType(tree);
         }
    };
    /// static member to ensure static initialization (only once)
    static StaticMemberInitializer capabilityOcTreeMemberInit;

  private:

    std::string _groupName;
    std::string _baseName;
    std::string _tipName;
};


#endif // CAPABILITYOCTREE_H
