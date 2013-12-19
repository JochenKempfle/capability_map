#include <gtest/gtest.h>
#include <octomap/OcTreeBase.h>
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
    CapabilityOcTree(double resolution) : OcTreeBase<CapabilityOcTreeNode>(resolution) {};

    // virtual constructor: creates a new object of same type
    // (Covariant return type requires an up-to-date compiler)
    CapabilityOcTree* create() const {return new CapabilityOcTree(resolution); }

    std::string getTreeType() const { return "CapabilityOcTree"; }


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

};


#endif // CAPABILITYOCTREE_H
