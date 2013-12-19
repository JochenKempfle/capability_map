#include <cmath>
#include "capability_map/CapabilityOcTreeNode.h"


bool Capability::isDirectionPossible(double phi, double theta) const
{
    assert(theta <= 180.0);

    // convert angles to rad
    double thetaInRad1 = M_PI * (90.0 - _theta) / 180.0;
    double thetaInRad2 = M_PI * (90.0 - theta) / 180.0;
    double phiInRad1 = M_PI * _phi / 180.0;
    double phiInRad2 = M_PI * phi / 180.0;
    // calculate the shortest angle (great-circle distance) between both directions
    double orthodromeAngle = acos(sin(thetaInRad1) * sin(thetaInRad2) + cos(thetaInRad1) *
                                  cos(thetaInRad2) * cos(phiInRad2 - phiInRad1)) * 180.0 / M_PI;

    bool retValue = false;

    switch (_type)
    {
        case EMPTY:
            // no direction possible
            retValue = false;
            break;

        case SPHERE:
            // every direction possible
            retValue = true;
            break;

        case CONE:
            // only directions are possible that lie inside the cone's opening angle
            retValue = orthodromeAngle <= _halfOpeningAngle;
            break;

        case CYLINDER_1:
            // CYLINDER_1 can be seen as double cone, possible directions must lie inside opening angle of both sides
            retValue = orthodromeAngle <= _halfOpeningAngle || std::abs(orthodromeAngle - 180.0) <= _halfOpeningAngle;
            break;

        case CYLINDER_2:
            // the sensitive area of the cylinder is orthogonal to the up-direction (up-direction +- 90°)
            retValue = std::abs(90.0 - orthodromeAngle) <= _halfOpeningAngle;
            break;

        default:
            retValue = false;
    }

    return retValue;
}

CapabilityOcTreeNode::CapabilityOcTreeNode() : OcTreeDataNode<Capability>(Capability(EMPTY, 0.0, 0.0, 0.0, 0.0))
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

