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


    FRIEND_TEST(CapabilityOcTreeTest, constructor);

    /// Default constructor, sets resolution of leafs
    CapabilityOcTree(double resolution) : OcTreeBase<CapabilityOcTreeNode>(resolution) {};

    /// virtual constructor: creates a new object of same type
    /// (Covariant return type requires an up-to-date compiler)
    CapabilityOcTree* create() const {return new CapabilityOcTree(resolution); }

    std::string getTreeType() const { return "CapabilityOcTree"; }


    FRIEND_TEST(CapabilityOcTreeTest, setNodeCapability);

    // set node capability at given key or coordinate. Replaces previous capability.
    CapabilityOcTreeNode* setNodeCapability(const OcTreeKey &key, const Capability &capability);

    CapabilityOcTreeNode* setNodeCapability(const float &x, const float &y,
                                            const float &z, CAPABILITY_TYPE type,
                                            double phi, double theta, double openingAngle)
    {
        OcTreeKey key;
        if (!this->coordToKeyChecked(point3d(x,y,z), key))
        {
            return NULL;
        }
        return setNodeCapability(key, Capability(type, phi, theta, openingAngle));
    }

    CapabilityOcTreeNode* setNodeCapability(const float &x, const float &y,
                                            const float &z, const Capability &capability)
    {
        OcTreeKey key;
        if (!this->coordToKeyChecked(point3d(x,y,z), key))
        {
            return NULL;
        }
        return setNodeCapability(key, capability);
    }


  protected:
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
