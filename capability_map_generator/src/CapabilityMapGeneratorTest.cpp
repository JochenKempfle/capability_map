#include <gtest/gtest.h>
#include <iostream>
#include <cmath>
#include <vector>
#include "capability_map_generator/ReachabilitySphere.h"

// helper function for floating point comparison
bool fuzzyEquals(double i, double j)
{
    return std::abs(i - j) < 0.00000001;
}

TEST(ReachabilitySphere, appendDirection)
{
    ReachabilitySphere sphere;

    sphere.appendDirection(0.0, 0.0, true);
    sphere.appendDirection(0.0, 90.0, false);

    sphere.appendDirection(0.0, 1.0, 0.0, true);
    sphere.appendDirection(0.0, 0.0, 1.0, false);

    ASSERT_TRUE(sphere._reachableDirections[0].x == 0.0);
    ASSERT_TRUE(sphere._reachableDirections[0].y == 0.0);
    ASSERT_TRUE(sphere._reachableDirections[0].z == 1.0);

    ASSERT_TRUE(sphere._unreachableDirections[0].x == 1.0);
    ASSERT_TRUE(sphere._unreachableDirections[0].y == 0.0);
    ASSERT_TRUE(fuzzyEquals(sphere._unreachableDirections[0].z, 0.0));

    ASSERT_TRUE(sphere._reachableDirections[1].x == 0.0);
    ASSERT_TRUE(sphere._reachableDirections[1].y == 1.0);
    ASSERT_TRUE(sphere._reachableDirections[1].z == 0.0);

    ASSERT_TRUE(sphere._unreachableDirections[1].x == 0.0);
    ASSERT_TRUE(sphere._unreachableDirections[1].y == 0.0);
    ASSERT_TRUE(sphere._unreachableDirections[1].z == 1.0);
}

TEST(ReachabilitySphere, getPrincipalComponents)
{
    ReachabilitySphere sphere;
    std::vector<ReachabilitySphere::Vector> vectors;
    std::vector<ReachabilitySphere::Vector> components;

    // a line in x direction should give the main principal component lying in x direction
    vectors.push_back(ReachabilitySphere::Vector(0.0, 0.0, 0.0));
    vectors.push_back(ReachabilitySphere::Vector(1.0, 0.0, 0.0));
    vectors.push_back(ReachabilitySphere::Vector(2.0, 0.0, 0.0));
    vectors.push_back(ReachabilitySphere::Vector(3.0, 0.0, 0.0));
    
    components = sphere.getPrincipalComponents(vectors);

    ASSERT_TRUE(components[2].x == 1.0);
    ASSERT_TRUE(components[2].y == 0.0);
    ASSERT_TRUE(components[2].z == 0.0);

    vectors.clear();

    // a line in y direction should give the main principal component lying in y direction
    vectors.push_back(ReachabilitySphere::Vector(0.0, 0.0, 0.0));
    vectors.push_back(ReachabilitySphere::Vector(0.0, 1.0, 0.0));
    vectors.push_back(ReachabilitySphere::Vector(0.0, 2.0, 0.0));
    vectors.push_back(ReachabilitySphere::Vector(0.0, 3.0, 0.0));
    
    components = sphere.getPrincipalComponents(vectors);

    ASSERT_TRUE(components[2].x == 0.0);
    ASSERT_TRUE(components[2].y == 1.0);
    ASSERT_TRUE(components[2].z == 0.0);

    vectors.clear();

    // a line in z direction should give the main principal component lying in z direction
    vectors.push_back(ReachabilitySphere::Vector(0.0, 0.0, 0.0));
    vectors.push_back(ReachabilitySphere::Vector(0.0, 0.0, 1.0));
    vectors.push_back(ReachabilitySphere::Vector(0.0, 0.0, 2.0));
    vectors.push_back(ReachabilitySphere::Vector(0.0, 0.0, 3.0));
    
    components = sphere.getPrincipalComponents(vectors);

    ASSERT_TRUE(components[2].x == 0.0);
    ASSERT_TRUE(components[2].y == 0.0);
    ASSERT_TRUE(components[2].z == 1.0);

    vectors.clear();

    // filling just x and y directions should give the least principal component in z direction
    vectors.push_back(ReachabilitySphere::Vector(1.0, 2.0, 0.0));
    vectors.push_back(ReachabilitySphere::Vector(2.0, 1.0, 0.0));
    vectors.push_back(ReachabilitySphere::Vector(1.0, 0.0, 0.0));
    vectors.push_back(ReachabilitySphere::Vector(0.0, 1.0, 0.0));
    
    components = sphere.getPrincipalComponents(vectors);

    ASSERT_TRUE(components[0].x == 0.0);
    ASSERT_TRUE(components[0].y == 0.0);
    ASSERT_TRUE(components[0].z == 1.0);

    vectors.clear();

    // filling just y and z directions should give the least principal component in x direction
    vectors.push_back(ReachabilitySphere::Vector(0.0, 2.0, 0.0));
    vectors.push_back(ReachabilitySphere::Vector(0.0, 1.0, 2.0));
    vectors.push_back(ReachabilitySphere::Vector(0.0, 0.0, 1.0));
    vectors.push_back(ReachabilitySphere::Vector(0.0, 1.0, 1.0));
    
    components = sphere.getPrincipalComponents(vectors);

    ASSERT_TRUE(components[0].x == 1.0);
    ASSERT_TRUE(components[0].y == 0.0);
    ASSERT_TRUE(components[0].z == 0.0);

    vectors.clear();

    // filling just x and z directions should give the least principal component in y direction
    vectors.push_back(ReachabilitySphere::Vector(1.0, 0.0, 0.0));
    vectors.push_back(ReachabilitySphere::Vector(2.0, 0.0, 2.0));
    vectors.push_back(ReachabilitySphere::Vector(2.0, 0.0, 1.0));
    vectors.push_back(ReachabilitySphere::Vector(0.0, 0.0, 1.0));
    
    components = sphere.getPrincipalComponents(vectors);

    ASSERT_TRUE(components[0].x == 0.0);
    ASSERT_TRUE(components[0].y == 1.0);
    ASSERT_TRUE(components[0].z == 0.0);

    vectors.clear();

    // a line in x-y direction should give the main principal component lying in x-y direction
    vectors.push_back(ReachabilitySphere::Vector(0.0, 0.0, 0.0));
    vectors.push_back(ReachabilitySphere::Vector(1.0, 1.0, 0.0));
    vectors.push_back(ReachabilitySphere::Vector(2.0, 2.0, 0.0));
    vectors.push_back(ReachabilitySphere::Vector(3.0, 3.0, 0.0));
    
    components = sphere.getPrincipalComponents(vectors);

    ASSERT_TRUE(fuzzyEquals(std::abs(components[2].x), 0.707106781));
    ASSERT_TRUE(fuzzyEquals(std::abs(components[2].x), 0.707106781));
    ASSERT_TRUE(components[2].z == 0.0);
}

TEST(ReachabilitySphere, fitCone)
{
    
}

TEST(ReachabilitySphere, fitCylinder_1)
{
    
}

TEST(ReachabilitySphere, fitCylinder_2)
{
    
}

TEST(ReachabilitySphere, convertToCapability)
{
    
}


int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

