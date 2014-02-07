#include <iostream>
#include <gtest/gtest.h>
#include "capability_map/CapabilityOcTreeNode.h"
#include "capability_map/CapabilityOcTree.h"


TEST(Capability, constructor)
{
    // check if constructors do their job
    Capability cap1;

    ASSERT_EQ(EMPTY, cap1._type);
    ASSERT_EQ(0.0, cap1._phi);
    ASSERT_EQ(0.0, cap1._theta);
    ASSERT_EQ(0.0, cap1._halfOpeningAngle);
    ASSERT_EQ(0.0, cap1._shapeFitError);

    Capability cap2(SPHERE, 0.1, 0.2, 0.3, 2.0);

    ASSERT_TRUE(SPHERE == cap2._type);
    ASSERT_TRUE(0.1 == cap2._phi);
    ASSERT_TRUE(0.2 == cap2._theta);
    ASSERT_TRUE(0.3 == cap2._halfOpeningAngle);
    ASSERT_TRUE(2.0 == cap2._shapeFitError);

    // test if shape fit error is correctly initialized to zero
    Capability cap3(CONE, 10.0, 10.0, 10.0);
    ASSERT_TRUE(0.0 == cap3._shapeFitError);
}

TEST(Capability, equalityOperators)
{
    // test equality operators with various kinds of data
    Capability cap1(SPHERE, 0.1, 0.2, 0.3);
    Capability cap2(SPHERE, 0.1, 0.2, 0.3);
    Capability cap3(SPHERE, 0.0, 0.1, 0.2);
    Capability cap4(CONE, 0.1, 0.2, 0.3);
    Capability cap5(CONE, 0.1, 0.2, 0.4);
    // test for shape fit error
    Capability cap6(CONE, 0.1, 0.2, 0.4, 2.0);

    ASSERT_TRUE(cap1 == cap2);
    ASSERT_FALSE(cap1 == cap3);
    ASSERT_FALSE(cap1 == cap4);
    ASSERT_FALSE(cap4 == cap5);
    ASSERT_FALSE(cap5 == cap6);

    ASSERT_FALSE(cap1 != cap2);
    ASSERT_TRUE(cap1 != cap3);
    ASSERT_TRUE(cap1 != cap4);
    ASSERT_TRUE(cap4 != cap5);
    ASSERT_TRUE(cap5 != cap6);
}

TEST(Capability, isDirectionPossible)
{
    // test isDirectionPossible() with all kinds of capabilities (rounding errors are considered)

    // test empty capability (expect false for all directions)
    Capability emptyCap(EMPTY, 0.0, 0.0, 0.0);

    ASSERT_FALSE(emptyCap.isDirectionPossible(0.0, 0.0));
    ASSERT_FALSE(emptyCap.isDirectionPossible(5.0, 10.0));

    // test sphere-like capability (expect true for all directions)
    Capability sphereCap(SPHERE, 0.0, 0.0, 0.0);

    ASSERT_TRUE(sphereCap.isDirectionPossible(0.0, 0.0));
    ASSERT_TRUE(sphereCap.isDirectionPossible(5.0, 10.0));

    // test cone-like capability (expect true if direction lies inside cone, false otherwise)
    Capability coneCap(CONE, 10.0, 30.0, 20.01);
    Capability coneCap2(CONE, 0.0, 90.0, 20.01);

    // test "inside corners"
    ASSERT_TRUE(coneCap.isDirectionPossible(10.0, 10.0));
    ASSERT_TRUE(coneCap.isDirectionPossible(10.0, 50.0));
    ASSERT_TRUE(coneCap.isDirectionPossible(30.0, 30.0));
    ASSERT_TRUE(coneCap.isDirectionPossible(350.0, 30.0));
    // test phi with theta = 90° rotated cone
    ASSERT_TRUE(coneCap2.isDirectionPossible(20.0, 90.0));
    ASSERT_TRUE(coneCap2.isDirectionPossible(340.01, 90.0));
    // test inside
    ASSERT_TRUE(coneCap.isDirectionPossible(10.0, 30.0));
    ASSERT_TRUE(coneCap.isDirectionPossible(20.0, 40.0));
    ASSERT_TRUE(coneCap.isDirectionPossible(0.0, 20.0));
    // test "outside corners"
    ASSERT_FALSE(coneCap.isDirectionPossible(10.0, 50.02));
    ASSERT_FALSE(coneCap.isDirectionPossible(10.0, 9.98));
    // test phi with theta = 90° rotated coneCap2
    ASSERT_FALSE(coneCap2.isDirectionPossible(20.02, 90.0));
    ASSERT_FALSE(coneCap2.isDirectionPossible(339.98, 90.0));
    // test outside
    ASSERT_FALSE(coneCap.isDirectionPossible(90.0, 90.0));
    ASSERT_FALSE(coneCap.isDirectionPossible(240.0, 160.0));

    // test cylinder_1-like capability (expect true if direction lies inside cylinder, false otherwise)
    Capability cylinder1Cap(CYLINDER_1, 30.0, 60.0, 20.01);

    // test "inside corners"
    ASSERT_TRUE(cylinder1Cap.isDirectionPossible(30.0, 40.0));
    ASSERT_TRUE(cylinder1Cap.isDirectionPossible(30.0, 80.0));
    ASSERT_TRUE(cylinder1Cap.isDirectionPossible(10.0, 60.0));
    ASSERT_TRUE(cylinder1Cap.isDirectionPossible(50.0, 60.0));
    ASSERT_TRUE(cylinder1Cap.isDirectionPossible(210.0, 140.0));
    ASSERT_TRUE(cylinder1Cap.isDirectionPossible(210.0, 100.0));
    ASSERT_TRUE(cylinder1Cap.isDirectionPossible(190.0, 120.0));
    ASSERT_TRUE(cylinder1Cap.isDirectionPossible(230.0, 120.0));
    // test inside cylinder
    ASSERT_TRUE(cylinder1Cap.isDirectionPossible(30.0, 60.0));
    ASSERT_TRUE(cylinder1Cap.isDirectionPossible(210.0, 120.0));
    // test phi and theta themself lie inside cylinder, but not combined as direction (corners)
    ASSERT_FALSE(cylinder1Cap.isDirectionPossible(50.0, 140.0));
    ASSERT_FALSE(cylinder1Cap.isDirectionPossible(50.0, 100.0));
    ASSERT_FALSE(cylinder1Cap.isDirectionPossible(10.0, 140.0));
    ASSERT_FALSE(cylinder1Cap.isDirectionPossible(10.0, 100.0));
    ASSERT_FALSE(cylinder1Cap.isDirectionPossible(230.0, 40.0));
    ASSERT_FALSE(cylinder1Cap.isDirectionPossible(230.0, 80.0));
    ASSERT_FALSE(cylinder1Cap.isDirectionPossible(190.0, 40.0));
    ASSERT_FALSE(cylinder1Cap.isDirectionPossible(190.0, 80.0));
    // test phi and theta themself lie inside cylinder, but not combined as direction
    ASSERT_FALSE(cylinder1Cap.isDirectionPossible(30.0, 120.0));
    ASSERT_FALSE(cylinder1Cap.isDirectionPossible(210.0, 60.0));
    // test "outside corners"
    ASSERT_FALSE(cylinder1Cap.isDirectionPossible(9.98, 39.98));
    ASSERT_FALSE(cylinder1Cap.isDirectionPossible(9.98, 80.02));
    ASSERT_FALSE(cylinder1Cap.isDirectionPossible(50.02, 39.98));
    ASSERT_FALSE(cylinder1Cap.isDirectionPossible(50.02, 80.02));
    ASSERT_FALSE(cylinder1Cap.isDirectionPossible(189.98, 99.98));
    ASSERT_FALSE(cylinder1Cap.isDirectionPossible(189.98, 140.02));
    ASSERT_FALSE(cylinder1Cap.isDirectionPossible(230.02, 99.98));
    ASSERT_FALSE(cylinder1Cap.isDirectionPossible(230.02, 140.02));
    // test outside
    ASSERT_FALSE(cylinder1Cap.isDirectionPossible(90.0, 90.0));
    ASSERT_FALSE(cylinder1Cap.isDirectionPossible(300.0, 150.0));

    // test cylinder_2-like capability (expect true if direction lies inside cylinder, false otherwise)
    Capability cylinder2Cap(CYLINDER_2, 0.0, 0.0, 20.01);

    // test various "inside corners"
    ASSERT_TRUE(cylinder2Cap.isDirectionPossible(0.0, 70.0));
    ASSERT_TRUE(cylinder2Cap.isDirectionPossible(0.0, 110.0));
    ASSERT_TRUE(cylinder2Cap.isDirectionPossible(50.0, 70.0));
    ASSERT_TRUE(cylinder2Cap.isDirectionPossible(90.0, 110.0));
    ASSERT_TRUE(cylinder2Cap.isDirectionPossible(130.0, 70.0));
    ASSERT_TRUE(cylinder2Cap.isDirectionPossible(190.0, 110.0));
    ASSERT_TRUE(cylinder2Cap.isDirectionPossible(250.0, 70.0));
    ASSERT_TRUE(cylinder2Cap.isDirectionPossible(310.0, 110.0));
    ASSERT_TRUE(cylinder2Cap.isDirectionPossible(350.0, 70.0));
    // test inside
    ASSERT_TRUE(cylinder2Cap.isDirectionPossible(0.0, 80.0));
    ASSERT_TRUE(cylinder2Cap.isDirectionPossible(110.0, 90.0));
    ASSERT_TRUE(cylinder2Cap.isDirectionPossible(220.0, 98.765));
    ASSERT_TRUE(cylinder2Cap.isDirectionPossible(330.0, 100.0));
    // test "outside corners"
    ASSERT_FALSE(cylinder2Cap.isDirectionPossible(0.0, 69.98));
    ASSERT_FALSE(cylinder2Cap.isDirectionPossible(0.0, 110.02));
    ASSERT_FALSE(cylinder2Cap.isDirectionPossible(110.0, 69.98));
    ASSERT_FALSE(cylinder2Cap.isDirectionPossible(220.0, 110.02));
    ASSERT_FALSE(cylinder2Cap.isDirectionPossible(330.0, 69.98));
    // test outside
    ASSERT_FALSE(cylinder2Cap.isDirectionPossible(0.0, 0.0));
    ASSERT_FALSE(cylinder2Cap.isDirectionPossible(20.0, 0.0));
    ASSERT_FALSE(cylinder2Cap.isDirectionPossible(0.0, 20.0));
    ASSERT_FALSE(cylinder2Cap.isDirectionPossible(50.0, 50.0));
    ASSERT_FALSE(cylinder2Cap.isDirectionPossible(320.0, 150.0));

    // test rotated cylinder_2-like capability (expect true if direction lies inside cylinder, false otherwise)
    Capability rotCylinder2Cap(CYLINDER_2, 0.0, 20.0, 20.01);

    // only test some special cases
    // test "inside corners"
    ASSERT_TRUE(rotCylinder2Cap.isDirectionPossible(0.0, 90.0));
    ASSERT_TRUE(rotCylinder2Cap.isDirectionPossible(0.0, 130.0));
    ASSERT_TRUE(rotCylinder2Cap.isDirectionPossible(180.0, 50.0));
    ASSERT_TRUE(rotCylinder2Cap.isDirectionPossible(180.0, 90.0));
    // test inside
    ASSERT_TRUE(rotCylinder2Cap.isDirectionPossible(0.0, 110.0));
    ASSERT_TRUE(rotCylinder2Cap.isDirectionPossible(10.0, 100.0));
    ASSERT_TRUE(rotCylinder2Cap.isDirectionPossible(180.0, 70.0));
    ASSERT_TRUE(rotCylinder2Cap.isDirectionPossible(190.0, 60.0));
    // test "outside corners"
    ASSERT_FALSE(rotCylinder2Cap.isDirectionPossible(0.0, 89.98));
    ASSERT_FALSE(rotCylinder2Cap.isDirectionPossible(0.0, 130.02));
    ASSERT_FALSE(rotCylinder2Cap.isDirectionPossible(180.0, 49.98));
    ASSERT_FALSE(rotCylinder2Cap.isDirectionPossible(180.0, 90.02));
    // test outside
    ASSERT_FALSE(rotCylinder2Cap.isDirectionPossible(0.0, 0.0));
    ASSERT_FALSE(rotCylinder2Cap.isDirectionPossible(0.0, 180.0));
    ASSERT_FALSE(rotCylinder2Cap.isDirectionPossible(50.0, 10.0));
}

TEST(Capability, getPercentReachable)
{
    // test empty capability (expect 0.0%)
    Capability emptyCap(EMPTY, 0.0, 0.0, 0.0);

    ASSERT_DOUBLE_EQ(0.0, emptyCap.getPercentReachable());

    // test sphere-like capability (expect 100.0%)
    Capability sphereCap(SPHERE, 0.0, 0.0, 0.0);

    ASSERT_DOUBLE_EQ(100.0, sphereCap.getPercentReachable());

    // test cone-like capability
    Capability coneCap(CONE, 10.0, 30.0, 90.0);
    Capability coneCap2(CONE, 0.0, 90.0, 60.0);

    ASSERT_DOUBLE_EQ(50.0, coneCap.getPercentReachable());
    ASSERT_DOUBLE_EQ(25.0, coneCap2.getPercentReachable());

    // test cylinder_1-like capability
    Capability cylinder_1Cap(CYLINDER_1, 10.0, 30.0, 60.0);
    //Capability cylinder_1Cap2(CYLINDER_1, 0.0, 90.0, 30.0);

    ASSERT_DOUBLE_EQ(50.0, cylinder_1Cap.getPercentReachable());
    //ASSERT_DOUBLE_EQ(25.0, cylinder_1Cap2.getPercentReachable());

    // test cylinder_2-like capability
    Capability cylinder_2Cap(CYLINDER_2, 10.0, 30.0, 30.0);

    ASSERT_DOUBLE_EQ(50.0, cylinder_2Cap.getPercentReachable());
}

TEST(CapabilityOcTreeNode, constructor)
{
    // check if constructors do their job
    Capability emptyCap(EMPTY, 0.0, 0.0, 0.0);
    Capability sphereCap(SPHERE, 0.1, 0.2, 0.3);


    CapabilityOcTreeNode node1;
    CapabilityOcTreeNode node2(sphereCap);
    CapabilityOcTreeNode node3(node2);

    ASSERT_TRUE(emptyCap == node1.getCapability());
    ASSERT_TRUE(sphereCap == node2.getCapability());
    ASSERT_TRUE(sphereCap == node3.getCapability());
}

TEST(CapabilityOcTreeNode, equalityOperator)
{
    // test operator== with various kinds of data
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
    // test set and get functions of CapabilityOcTreeNode
    Capability cap(CYLINDER_2, 0.4, 0.0, 1.2);
    CapabilityOcTreeNode node1;
    CapabilityOcTreeNode node2;

    node1.setCapability(cap);
    node2.setCapability(CYLINDER_2, 0.4, 0.0, 1.2);

    ASSERT_TRUE(cap == node1.getCapability());
    ASSERT_TRUE(cap == node2.getCapability());
}


TEST(CapabilityOcTree, constructor)
{
    // does the constructor construct a CapabilityOcTree?
    CapabilityOcTree tree(0.1);
    ASSERT_EQ("CapabilityOcTree", tree.getTreeType());
}

TEST(CapabilityOcTree, set_getNodeCapability)
{
    // test set and get functions of CapabilityOcTree
    Capability emptyCap;
    Capability cap1(SPHERE, 0.1, 0.1, 0.1);
    Capability cap2(CONE, 0.2, 0.2, 0.2);
    Capability cap3(CYLINDER_1, 0.3, 0.3, 0.3);
    Capability cap4(CYLINDER_2, 0.4, 0.4, 0.4);

    CapabilityOcTreeNode* testNode = NULL;

    CapabilityOcTree tree(0.1);
    testNode = tree.setNodeCapability(1.0, 1.0, 1.0, SPHERE, 0.1, 0.1, 0.1);  // == cap1
    ASSERT_FALSE(testNode == NULL);
    
    tree.setNodeCapability(1.0, 1.0, 2.0, Capability(CONE, 0.2, 0.2, 0.2));  // == cap2
    ASSERT_FALSE(testNode == NULL);
    
    tree.setNodeCapability(1.2, 1.0, 1.0, cap3);
    ASSERT_FALSE(testNode == NULL);
    
    tree.setNodeCapability(1.0, 1.2, 1.0, cap4);
    ASSERT_FALSE(testNode == NULL);

    tree.setNodeCapability(1.0, 2.0, 1.0, cap4);

    // test if all capabilities are inserted correctly
    ASSERT_TRUE(cap1 == tree.getNodeCapability(1.0, 1.0, 1.0));
    ASSERT_TRUE(cap2 == tree.getNodeCapability(1.0, 1.0, 2.0));
    ASSERT_TRUE(cap3 == tree.getNodeCapability(1.2, 1.0, 1.0));
    ASSERT_TRUE(cap4 == tree.getNodeCapability(1.0, 1.2, 1.0));
    ASSERT_TRUE(cap4 == tree.getNodeCapability(1.0, 2.0, 1.0));

    // replace cap3 with cap4
    tree.setNodeCapability(1.2, 1.0, 1.0, cap4);

    ASSERT_TRUE(cap4 == tree.getNodeCapability(1.2, 1.0, 1.0));

    // test for not inserted nodes
    ASSERT_TRUE(emptyCap == tree.getNodeCapability(2.0, 2.0, 2.0));
}

TEST(CapabilityOcTree, isPosePossible)
{
    // test if isPosePossible() returns correct value (more accurate tests are in TEST(Capability, isDirectionPossible))
    CapabilityOcTree tree(0.1);
    tree.setNodeCapability(1.0, 1.0, 1.0, CONE, 20.0, 20.0, 10.1);

    // testing just two nodes is enough to ensure this function is working
    ASSERT_TRUE(tree.isPosePossible(1.0, 1.0, 1.0, 20.0, 20.0));
    ASSERT_TRUE(tree.isPosePossible(1.0, 1.0, 1.0, 20.0, 25.0));
    ASSERT_FALSE(tree.isPosePossible(1.0, 1.0, 1.0, 50.0, 50.0));

    ASSERT_FALSE(tree.isPosePossible(2.0, 2.0, 2.0, 0.0, 0.0));
}

TEST(CapabilityOcTree, read_writeBinary)
{
    CapabilityOcTree tree(0.1);
    tree.setNodeCapability(1.0, 1.0, 1.0, EMPTY, 0.0, 0.0, 0.0);
    tree.setNodeCapability(1.2, 1.0, 1.0, SPHERE, 20.0, 20.0, 10.0);
    tree.setNodeCapability(1.2, 1.2, 1.0, CONE, 20.0, 20.0, 10.0);
    tree.setNodeCapability(2.0, 1.0, 1.0, CYLINDER_1, 20.0, 20.0, 10.0);
    tree.setNodeCapability(2.0, 2.0, 1.0, CYLINDER_2, 20.0, 20.0, 10.0);

    tree.setGroupName("group");
    tree.setBaseName("base");
    tree.setTipName("tip");

    tree.writeFile("./test_tree.cpm");

    CapabilityOcTree* tree2 = CapabilityOcTree::readFile("./test_tree.cpm");

    remove("./test_tree.cpm");

    ASSERT_TRUE(tree.getNodeCapability(1.0, 1.0, 1.0) == tree2->getNodeCapability(1.0, 1.0, 1.0));
    ASSERT_TRUE(tree.getNodeCapability(1.2, 1.0, 1.0) == tree2->getNodeCapability(1.2, 1.0, 1.0));
    ASSERT_TRUE(tree.getNodeCapability(1.2, 1.2, 1.0) == tree2->getNodeCapability(1.2, 1.2, 1.0));
    ASSERT_TRUE(tree.getNodeCapability(2.0, 1.0, 1.0) == tree2->getNodeCapability(2.0, 1.0, 1.0));
    ASSERT_TRUE(tree.getNodeCapability(2.0, 2.0, 1.0) == tree2->getNodeCapability(2.0, 2.0, 1.0));

    ASSERT_EQ(tree.getGroupName(), tree2->getGroupName());
    ASSERT_EQ(tree.getBaseName(), tree2->getBaseName());
    ASSERT_EQ(tree.getTipName(), tree2->getTipName());

    delete tree2;

    // test if empty, whitespaced or weird group, base and tip names are written and read
    tree.setGroupName(" /~#_*+:|<> ");
    tree.setBaseName(" ");
    tree.setTipName("");

    tree.writeFile("./test_tree.cpm");

    tree2 = CapabilityOcTree::readFile("./test_tree.cpm");

    remove("./test_tree.cpm");

    ASSERT_EQ(tree.getGroupName(), tree2->getGroupName());
    ASSERT_EQ(tree.getBaseName(), tree2->getBaseName());
    ASSERT_EQ(tree.getTipName(), tree2->getTipName());

    delete tree2;
}



int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
