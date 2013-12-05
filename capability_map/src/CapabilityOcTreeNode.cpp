#include "./../include/capability_map/CapabilityOcTreeNode.h"



CapabilityOcTreeNode::CapabilityOcTreeNode() : OcTreeDataNode<Capability>(Capability(EMPTY, 0.0f, 0.0f, 0.0f))
{

}

CapabilityOcTreeNode::CapabilityOcTreeNode(Capability capability) : OcTreeDataNode<Capability>(capability)
{

}

CapabilityOcTreeNode::~CapabilityOcTreeNode()
{

}

bool CapabilityOcTreeNode::createChild(unsigned int i)
{
    if (children == NULL)
    {
        allocChildren();
    }
    assert (children[i] == NULL);
    children[i] = new CapabilityOcTreeNode();
    return true;
}

/*
std::ostream& CapabilityOcTreeNode::writeValue(std::ostream &s) const
{
    // 1 bit for each children; 0: empty, 1: allocated
    std::bitset<8> children;
    for (unsigned int i = 0; i < 8; ++i)
    {
        if (childExists(i))
        {
          children[i] = 1;
        }
        else
        {
          children[i] = 0;
        }
    }
    char children_char = (char)children.to_ulong();

    // write node data
    s.write((const char*)&value, sizeof(value)); // occupancy
    s.write((const char*)&_capability, sizeof(Capability)); // capability
    s.write((char*)&children_char, sizeof(char)); // child existence

    // write existing children
    for (unsigned int i = 0; i < 8; ++i)
    {
        if (children[i] == 1)
        {
            this->getChild(i)->writeValue(s);
        }
    }
    return s;
}

std::istream& CapabilityOcTreeNode::readValue(std::istream &s)
{
    // read node data
    char children_char;
    s.read((char*)&value, sizeof(value)); // occupancy
    s.read((char*)&_capability, sizeof(Capability)); // capability
    s.read((char*)&children_char, sizeof(char)); // child existence

    // read existing children
    std::bitset<8> children ((unsigned long long)children_char);
    for (unsigned int i = 0; i < 8; ++i)
    {
        if (children[i] == 1)
        {
            createChild(i);
            getChild(i)->readValue(s);
        }
    }
    return s;
}
*/

