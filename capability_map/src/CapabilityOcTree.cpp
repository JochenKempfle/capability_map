#include "capability_map/CapabilityOcTree.h"
#include "capability_map/CapabilityOcTreeNode.h"


bool CapabilityOcTree::writeFile(const std::string &filename)
{
    std::ofstream file(filename.c_str(), std::ios_base::out | std::ios_base::binary);

    if (!file.is_open())
    {
        OCTOMAP_ERROR_STR("Filestream to " << filename << " not open, nothing written.");
        return false;
    }
    else
    {
        if (!write(file))
        {
            file.close();
            OCTOMAP_ERROR_STR("Could not write to " << filename);
            return false;
        }
        file << std::endl << "group_name " << _groupName << std::endl;
        file << "base_name " << _baseName << std::endl;
        file << "tip_name " << _tipName << std::endl;
        file.close();
    }
    return true;
}

CapabilityOcTree* CapabilityOcTree::readFile(const std::string &filename)
{
    std::ifstream file(filename.c_str(), std::ios_base::in |std::ios_base::binary);

    if (!file.is_open())
    {
        OCTOMAP_ERROR_STR("Filestream to " << filename << " not open, nothing read.");
        return NULL;
    }

    AbstractOcTree* abstractTree = AbstractOcTree::read(file);

    if (abstractTree == NULL || abstractTree->getTreeType() != "CapabilityOcTree")
    {
        OCTOMAP_ERROR_STR("Could not read " << filename << ". Is the file a valid capability map?");
        return NULL;
    }

    std::string qualifier;
    std::string groupName;
    std::string baseName;
    std::string tipName;

    while(!file.eof())
    {
        file >> qualifier;
        if (qualifier == "group_name")
        {
            file.ignore(1, ' ');
            std::getline(file, groupName);
        }
        else if (qualifier == "base_name")
        {
            file.ignore(1, ' ');
            std::getline(file, baseName);
        }
        else if (qualifier == "tip_name")
        {
            file.ignore(1, ' ');
            std::getline(file, tipName);
        }
    }

    file.close();

    CapabilityOcTree* tree = dynamic_cast<CapabilityOcTree*>(abstractTree);

    tree->setGroupName(groupName);
    tree->setBaseName(baseName);
    tree->setTipName(tipName);

    return tree;
}

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
    double correctionValue = resolution/100.0;
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
    double correctionValue = resolution/100.0;
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
    double correctionValue = resolution/100.0;
    return search(x + correctionValue, y + correctionValue, z + correctionValue)->getCapability();
}

bool CapabilityOcTree::isPosePossible(const double &x, const double &y, const double &z, const double &phi, const double &theta) const
{
    return search(x, y, z)->getCapability().isDirectionPossible(phi, theta);
}

bool CapabilityOcTree::isPosePossible(const octomap::pose6d pose) const
{
    octomath::Vector3 rotatedVector = pose.rot().rotate(octomath::Vector3(1.0, 0.0, 0.0));
    double phi = atan2(rotatedVector.y(), rotatedVector.x()) * 180.0 / M_PI;
    double theta = acos(rotatedVector.z()) * 180.0 / M_PI;

    return search(pose.x(), pose.y(), pose.z())->getCapability().isDirectionPossible(phi, theta);
}

std::vector<octomath::Vector3> CapabilityOcTree::getPositionsWithMinReachablePercent(double percent)
{
    std::vector<octomath::Vector3> positions;

    // loop through all capabilities
    for (CapabilityOcTree::leaf_iterator it = this->begin_leafs(), end = this->end_leafs(); it != end; ++it)
    {
        if (it->getCapability().getPercentReachable() < percent)
        {
            continue;
        }
        positions.push_back(octomath::Vector3(it.getX(), it.getY(), it.getZ()));
    }
    return positions;
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
