#include "./../include/capability_map/CapabilityOcTree.h"
#include "./../include/capability_map/CapabilityOcTreeNode.h"


CapabilityOcTreeNode* CapabilityOcTree::setNodeCapability(const OcTreeKey &key, const Capability &capability)
{
    CapabilityOcTreeNode* n = search(key);
    if (n != 0)
    {
        n->setValue(capability);
    }
    return n;
}

CapabilityOcTree::StaticMemberInitializer CapabilityOcTree::capabilityOcTreeMemberInit;
