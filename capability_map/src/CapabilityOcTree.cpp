#include "capability_map/CapabilityOcTree.h"
#include "capability_map/CapabilityOcTreeNode.h"


CapabilityOcTreeNode* CapabilityOcTree::setNodeCapability(const OcTreeKey &key, const Capability &capability)
{
    bool createdRoot = false;
    if (this->root == NULL){
      this->root = new CapabilityOcTreeNode();
      this->tree_size++;
      createdRoot = true;
    }

    return setNodeCapabilityRecurs(this->root, createdRoot, key, 0, capability);
}

CapabilityOcTreeNode* CapabilityOcTree::setNodeCapability(const double &x, const double &y, const double &z,
                                                          CAPABILITY_TYPE type, double phi, double theta,
                                                          double halfOpeningAngle, double shapeFitError)
{
    OcTreeKey key;
    // NOTE: there is a bug in creating a key. Floating point precision seems to cause the error
    // adding a small amount (1% of resolution) to x, y and z should handle this
    double correctionValue = resolution/100;
    if (!this->coordToKeyChecked(x + correctionValue, y + correctionValue, z + correctionValue, key))
    {
        return NULL;
    }
    return setNodeCapability(key, Capability(type, phi, theta, halfOpeningAngle, shapeFitError));
}

CapabilityOcTreeNode* CapabilityOcTree::setNodeCapability(const double &x, const double &y,
                                                          const double &z, const Capability &capability)
{
    OcTreeKey key;
    // NOTE: there is a bug in creating a key. Floating point precision seems to cause the error
    // adding a small amount (1% of resolution) to x, y and z should handle this
    double correctionValue = resolution/100;
    if (!this->coordToKeyChecked(x + correctionValue, y + correctionValue, z + correctionValue, key))
    {
        return NULL;
    }
    return setNodeCapability(key, capability);
}


Capability CapabilityOcTree::getNodeCapability(const double &x, const double &y, const double &z) const
{
    // NOTE: there is a bug in creating a key. Floating point precision seems to cause the error
    // adding a small amount (1% of resolution) to x, y and z should handle this
    double correctionValue = resolution/100;
    return search(x + correctionValue, y + correctionValue, z + correctionValue)->getCapability();
}

bool CapabilityOcTree::isPosePossible(const double &x, const double &y, const double &z, const double &phi, const double &theta) const
{
    return search(x, y, z)->getCapability().isDirectionPossible(phi, theta);
}

CapabilityOcTreeNode* CapabilityOcTree::setNodeCapabilityRecurs(CapabilityOcTreeNode* node, bool node_just_created,
                                                    const OcTreeKey& key, unsigned int depth, const Capability &capability)
{
    unsigned int pos = computeChildIdx(key, this->tree_depth -1 - depth);
    bool created_node = false;

    assert(node);

    // follow down to last level
    if (depth < this->tree_depth)
    {
        if (!node->childExists(pos))
        {
            // child does not exist, but maybe it's a pruned node?
            if ((!node->hasChildren()) && !node_just_created)
            {
                // current node does not have children AND it is not a new node
                // -> expand pruned node
                node->expandNode();
                this->tree_size += 8;
                this->size_changed = true;
            }
            else
            {
                // not a pruned node, create requested child
                node->createChild(pos);
                this->tree_size++;
                this->size_changed = true;
                created_node = true;
            }
        }

        CapabilityOcTreeNode* retval = setNodeCapabilityRecurs(node->getChild(pos), created_node, key, depth+1, capability);
        // prune node if possible, otherwise set own probability
        // note: combining both did not lead to a speedup!
        if (node->pruneNode())
        {
            this->tree_size -= 8;
            // return pointer to current parent (pruned), the just updated node no longer exists
            retval = node;
        }
        else
        {
            // TODO: maybe set some kind of maxCapability
            node->setCapability(capability);
        }

        return retval;
    }

    // at last level, update node, end of recursion
    else
    {
        node->setCapability(capability);
        return node;
    }
}

CapabilityOcTree::StaticMemberInitializer CapabilityOcTree::capabilityOcTreeMemberInit;
