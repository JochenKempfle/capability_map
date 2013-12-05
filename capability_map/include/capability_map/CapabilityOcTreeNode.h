#include <gtest/gtest.h>
#include <octomap/OcTreeDataNode.h>

#ifndef CAPABILITYOCTREENODE_H
#define CAPABILITYOCTREENODE_H

using namespace octomap;

enum CAPABILITY_TYPE
{
    EMPTY,
    SPHERE,
    CONE,
    CYLINDER_1,
    CYLINDER_2
};

/* **************************************************************************************************************************
 *                                                                                                                          *
 * Description of CAPABILITY_TYPE: Arrows show the possible directions allowed by this type (SPHERE allows all directions)  *
 *                                                                                                                          *
 *                                                                                                                          *
 *             x  x                    x*-._                             ________                         ,*"""*.           *
 *           x      x           -->   x  x  "-._                   -->  /\       \  <--             -->  |*.___.*|  <--     *
 * SPHERE:  x        x   CONE:  -->  x    x     "-._    CYLINDER1: --> (  )       ) <--  CYLINDER2: -->  |       |  <--     *
 *          x        x          -->  x    x    _.-"                -->  \/_______/  <--             -->  |       |  <--     *
 *           x      x           -->   x  x _.-"                                                           *-----*           *
 *             x  x                    x.-"                                                                                 *
 *                                                                                                                          *
 ***************************************************************************************************************************/



class Capability
{
  public:

    FRIEND_TEST(Capability, constructor);

    Capability() : _type(EMPTY), _phi(0.0f), _theta(0.0f), _openingAngle(0.0f) { }
    Capability(CAPABILITY_TYPE type, double phi, double theta, double openingAngle)
               : _type(type), _phi(phi), _theta(theta), _openingAngle(openingAngle) { }

    // TODO: are setter and getter needed? Uncomment if yes.
    /*
    void setType(CAPABILITY_TYPE type) { _type = type; }
    CAPABILITY_TYPE getType() const { return _type; }

    void setDirection(double phi, double theta) { _phi = phi; _theta = theta; }
    double getPhi() const { return _phi; }
    double getTheta() const { return _theta; }

    void setOpeningAngle(double openingAngle) { _openingAngle = openingAngle; }
    double getOpeningAngle() const { return _openingAngle; }
    */

    // TODO: implement isDirectionPossible (maybe rename function)
    bool isDirectionPossible(double phi, double theta) const;


    FRIEND_TEST(Capability, equalityOperators);

    inline bool operator==(const Capability &other) const
    {
        return (_type == other._type && _phi == other._phi && _theta == other._theta && _openingAngle == other._openingAngle);
    }
    inline bool operator!=(const Capability &other) const
    {
        return (_type != other._type || _phi != other._phi || _theta != other._theta || _openingAngle != other._openingAngle);
    }

  protected:

    // TODO: add Shape Fit Error
    CAPABILITY_TYPE _type;
    // the direction of the capability shape in sphere coordinates
    double _phi, _theta;
    // _openingAngle is the half opening angle of a cone or the height of a cylinder
    double _openingAngle;
};


class CapabilityOcTreeNode : public OcTreeDataNode<Capability>
{
  public:

    FRIEND_TEST(CapabilityOcTreeNode, constructor);

    CapabilityOcTreeNode();
    CapabilityOcTreeNode(Capability capability);

    ~CapabilityOcTreeNode();

    CapabilityOcTreeNode(const CapabilityOcTreeNode &rhs) : OcTreeDataNode<Capability>(rhs) { }

    FRIEND_TEST(CapabilityOcTreeNode, equalityOperator);

    bool operator==(const CapabilityOcTreeNode &rhs) const
    {
        return (rhs.value == value);
    }

    // children

    FRIEND_TEST(CapabilityOcTreeNode, children);

    bool createChild(unsigned int i);

    inline CapabilityOcTreeNode* getChild(unsigned int i)
    {
        return static_cast<CapabilityOcTreeNode*>(OcTreeDataNode<Capability>::getChild(i));
    }

    inline const CapabilityOcTreeNode* getChild(unsigned int i) const
    {
        return static_cast<const CapabilityOcTreeNode*>(OcTreeDataNode<Capability>::getChild(i));
    }


    // TODO: not really needed, node only gets pruned when value is equal for all children
    bool collapsible() { return false; }
    bool pruneNode() { return false; }
    void expandNode() { }

    FRIEND_TEST(CapabilityOcTreeNode, set_getCapability);

    inline Capability getCapability() const { return value; }
    inline void setCapability(Capability capability) { value = capability; }
    inline void setCapability(CAPABILITY_TYPE type, double phi, double theta, double openingAngle)
    {
        value = Capability(type, phi, theta, openingAngle);
    }

    // TODO: needed? If yes, uncomment
    // has a capability been set?
    /*
    inline bool isCapabilitySet() const
    {
        return (value.getType() != EMPTY);
    }
    */

    // void updateCapabilityChildren();


    // file I/O (should be covered by base class)
    // std::istream& readValue (std::istream &s);
    // std::ostream& writeValue(std::ostream &s) const;

  protected:

    // Capability value;
};

#endif // CAPABILITYOCTREENODE_H
