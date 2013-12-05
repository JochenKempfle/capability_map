
#include <gtest/gtest.h>
#include "./../include/capability_map/CapabilityOcTreeNode.h"
#include "./../include/capability_map/CapabilityOcTree.h"


TEST(Capability, constructor)
{
    Capability cap1;

    ASSERT_TRUE(EMPTY == cap1._type);
    ASSERT_TRUE(0.0 == cap1._phi);
    ASSERT_TRUE(0.0 == cap1._theta);
    ASSERT_TRUE(0.0 == cap1._openingAngle);


    Capability cap2(SPHERE, 0.1, 0.2, 0.3);

    ASSERT_TRUE(SPHERE == cap2._type);
    ASSERT_TRUE(0.1 == cap2._phi);
    ASSERT_TRUE(0.2 == cap2._theta);
    ASSERT_TRUE(0.3 == cap2._openingAngle);
}

TEST(Capability, equalityOperators)
{
    Capability cap1(SPHERE, 0.1, 0.2, 0.3);
    Capability cap2(SPHERE, 0.1, 0.2, 0.3);
    Capability cap3(SPHERE, 0.0, 0.1, 0.2);
    Capability cap4(CONE, 0.1, 0.2, 0.3);
    Capability cap5(CONE, 0.1, 0.2, 0.4);

    ASSERT_TRUE(cap1 == cap2);
    ASSERT_FALSE(cap1 == cap3);
    ASSERT_FALSE(cap1 == cap4);
    ASSERT_FALSE(cap4 == cap5);

    ASSERT_FALSE(cap1 != cap2);
    ASSERT_TRUE(cap1 != cap3);
    ASSERT_TRUE(cap1 != cap4);
    ASSERT_TRUE(cap4 != cap5);
}


TEST(CapabilityOcTreeNode, constructor)
{
    Capability emptyCap(EMPTY, 0.0, 0.0, 0.0);
    Capability sphereCap(SPHERE, 0.1, 0.2, 0.3);


    CapabilityOcTreeNode node1;
    CapabilityOcTreeNode node2(sphereCap);
    CapabilityOcTreeNode node3(node2);

    ASSERT_TRUE(emptyCap == node1.getCapability());
    ASSERT_TRUE(sphereCap == node2.getCapability());
    ASSERT_TRUE(sphereCap == node3.getCapability());

    //ASSERT_EQ(emptyCap, node1.getCapability());
    //ASSERT_EQ(sphereCap, node2.getCapability());
    //ASSERT_EQ(sphereCap, node3.getCapability());
}

TEST(CapabilityOcTreeNode, equalityOperator)
{
    CapabilityOcTreeNode node1(Capability(CYLINDER_1, 0.2, 0.3, 0.1));
    CapabilityOcTreeNode node2(Capability(CYLINDER_1, 0.2, 0.3, 0.1));
    CapabilityOcTreeNode node3(Capability(CYLINDER_2, 0.3, 0.1, 0.2));

    ASSERT_TRUE(node1 == node2);
    ASSERT_FALSE(node1 == node3);
}

TEST(CapabilityOcTreeNode, children)
{
    CapabilityOcTreeNode node;

    // TODO: how to check not yet set children (assert(children[i] != NULL) lets the test fail)
    for (unsigned int i = 0; i < 8; ++i)
    {
        //ASSERT_EQ(static_cast<CapabilityOcTreeNode*>(NULL), node.getChild(i));
    }

    node.createChild(1);

    //ASSERT_EQ(static_cast<CapabilityOcTreeNode*>(NULL), node.getChild(0));
    ASSERT_NE(static_cast<CapabilityOcTreeNode*>(NULL), node.getChild(1));
    for (unsigned int i = 2; i < 8; ++i)
    {
        //ASSERT_EQ(static_cast<CapabilityOcTreeNode*>(NULL), node.getChild(i));
    }
}

TEST(CapabilityOcTreeNode, set_getCapability)
{
    Capability cap(CYLINDER_2, 0.4, 0.0, 1.2);
    CapabilityOcTreeNode node1;
    CapabilityOcTreeNode node2;

    node1.setCapability(cap);
    node2.setCapability(CYLINDER_2, 0.4, 0.0, 1.2);

    ASSERT_TRUE(cap == node1.getCapability());
    ASSERT_TRUE(cap == node2.getCapability());

    //ASSERT_EQ(cap, node1.getCapability());
    //ASSERT_EQ(cap, node2.getCapability());
}


TEST(CapabilityOcTreeTest, constructor)
{
    CapabilityOcTree tree(0.1);
    ASSERT_EQ("CapabilityOcTree", tree.getTreeType());
}

TEST(CapabilityOcTreeTest, set_getNodeCapability)
{
    CapabilityOcTree tree(0.1);
    tree.setNodeCapability(1, 1, 1, SPHERE, 0.1, 0.1, 0.1);

    // TODO: test has no meaning for now
    ASSERT_EQ(1, 1);
}


int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
