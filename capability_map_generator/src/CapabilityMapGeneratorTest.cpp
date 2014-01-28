#include <gtest/gtest.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <utility>
#include "capability_map_generator/ReachabilitySphere.h"
#include "capability_map_generator/Vector.h"
#include "capability_map/CapabilityOcTreeNode.h"

namespace capability_map_generator
{

// helper function for floating point comparison
bool fuzzyEquals(double i, double j)
{
    return std::abs(i - j) < 0.00000001;
}

// function which equally distibutes points on a sphere
std::vector<Vector> distributePointsOnSphere(unsigned int n)
{
    // variant of spiral point algorithm (Saff et al.) which uses golden angle to spread points
    double goldenAngle = M_PI * (3.0 - sqrt(5.0));
    double dz = 2.0 / n;
    double angle = 0.0;
    double z = 1.0 - dz / 2.0;
    
    std::vector<Vector> points(n);
    
    for (unsigned int i = 0; i < n; ++i)
    {
        double r = sqrt(1.0 - z*z);
        points[i] = Vector(cos(angle) * r, sin(angle) * r, z);
        z = z - dz;
        angle = angle + goldenAngle;
    }

    return points;
}

ReachabilitySphere createReachabilitySphereFromCapability(Capability cap, unsigned int numPoints)
{
    // spread points over a sphere
    std::vector<Vector> points = distributePointsOnSphere(numPoints);
    
    ReachabilitySphere sphere;
    
    for (size_t i = 0; i < points.size(); ++i)
    {
        double phi = atan2(points[i].y, points[i].x) * 180.0 / M_PI;
        double theta = acos(points[i].z) * 180.0 / M_PI;
        
        if (cap.isDirectionPossible(phi, theta))
        {
            sphere.appendDirection(points[i].x, points[i].y, points[i].z, true);
        }
        else
        {
            sphere.appendDirection(points[i].x, points[i].y, points[i].z, false);
        }
    }
    return sphere;
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
    std::vector<Vector> vectors;
    std::vector<Vector> components;

    // a line in x direction should give the main principal component lying in x direction
    vectors.push_back(Vector(0.0, 0.0, 0.0));
    vectors.push_back(Vector(1.0, 0.0, 0.0));
    vectors.push_back(Vector(2.0, 0.0, 0.0));
    vectors.push_back(Vector(3.0, 0.0, 0.0));
    
    components = sphere.getPrincipalComponents(vectors);

    ASSERT_TRUE(components[2].x == 1.0);
    ASSERT_TRUE(components[2].y == 0.0);
    ASSERT_TRUE(components[2].z == 0.0);

    vectors.clear();

    // a line in y direction should give the main principal component lying in y direction
    vectors.push_back(Vector(0.0, 0.0, 0.0));
    vectors.push_back(Vector(0.0, 1.0, 0.0));
    vectors.push_back(Vector(0.0, 2.0, 0.0));
    vectors.push_back(Vector(0.0, 3.0, 0.0));
    
    components = sphere.getPrincipalComponents(vectors);

    ASSERT_TRUE(components[2].x == 0.0);
    ASSERT_TRUE(components[2].y == 1.0);
    ASSERT_TRUE(components[2].z == 0.0);

    vectors.clear();

    // a line in z direction should give the main principal component lying in z direction
    vectors.push_back(Vector(0.0, 0.0, 0.0));
    vectors.push_back(Vector(0.0, 0.0, 1.0));
    vectors.push_back(Vector(0.0, 0.0, 2.0));
    vectors.push_back(Vector(0.0, 0.0, 3.0));
    
    components = sphere.getPrincipalComponents(vectors);

    ASSERT_TRUE(components[2].x == 0.0);
    ASSERT_TRUE(components[2].y == 0.0);
    ASSERT_TRUE(components[2].z == 1.0);

    vectors.clear();

    // filling just x and y directions should give the least principal component in z direction
    vectors.push_back(Vector(1.0, 2.0, 0.0));
    vectors.push_back(Vector(2.0, 1.0, 0.0));
    vectors.push_back(Vector(1.0, 0.0, 0.0));
    vectors.push_back(Vector(0.0, 1.0, 0.0));
    
    components = sphere.getPrincipalComponents(vectors);

    ASSERT_TRUE(components[0].x == 0.0);
    ASSERT_TRUE(components[0].y == 0.0);
    ASSERT_TRUE(components[0].z == 1.0);

    vectors.clear();

    // filling just y and z directions should give the least principal component in x direction
    vectors.push_back(Vector(0.0, 2.0, 0.0));
    vectors.push_back(Vector(0.0, 1.0, 2.0));
    vectors.push_back(Vector(0.0, 0.0, 1.0));
    vectors.push_back(Vector(0.0, 1.0, 1.0));
    
    components = sphere.getPrincipalComponents(vectors);

    ASSERT_TRUE(components[0].x == 1.0);
    ASSERT_TRUE(components[0].y == 0.0);
    ASSERT_TRUE(components[0].z == 0.0);

    vectors.clear();

    // filling just x and z directions should give the least principal component in y direction
    vectors.push_back(Vector(1.0, 0.0, 0.0));
    vectors.push_back(Vector(2.0, 0.0, 2.0));
    vectors.push_back(Vector(2.0, 0.0, 1.0));
    vectors.push_back(Vector(0.0, 0.0, 1.0));
    
    components = sphere.getPrincipalComponents(vectors);

    ASSERT_TRUE(components[0].x == 0.0);
    ASSERT_TRUE(components[0].y == 1.0);
    ASSERT_TRUE(components[0].z == 0.0);

    vectors.clear();

    // a line in x-y direction should give the main principal component lying in x-y direction
    vectors.push_back(Vector(0.0, 0.0, 0.0));
    vectors.push_back(Vector(1.0, 1.0, 0.0));
    vectors.push_back(Vector(2.0, 2.0, 0.0));
    vectors.push_back(Vector(3.0, 3.0, 0.0));
    
    components = sphere.getPrincipalComponents(vectors);

    ASSERT_TRUE(fuzzyEquals(std::abs(components[2].x), 0.707106781));
    ASSERT_TRUE(fuzzyEquals(std::abs(components[2].y), 0.707106781));
    ASSERT_TRUE(components[2].z == 0.0);


    // test Capability of type cone
    Capability cone(CONE, 70.0, 55.0, 35.0);
    
    sphere = createReachabilitySphereFromCapability(cone, 400);

    components = sphere.getPrincipalComponents(sphere._reachableDirections);

    // test phi
    ASSERT_TRUE(atan2(components[0].y, components[0].x) * 180.0 / M_PI < 72.0);
    ASSERT_TRUE(atan2(components[0].y, components[0].x) * 180.0 / M_PI > 68.0);

    // test theta
    ASSERT_TRUE(acos(components[0].z) * 180.0 / M_PI < 56.0);
    ASSERT_TRUE(acos(components[0].z) * 180.0 / M_PI > 54.0);


    // test Capability of type cylinder_1
    Capability cylinder1(CYLINDER_1, 70.0, 55.0, 35.0);
    
    sphere = createReachabilitySphereFromCapability(cylinder1, 400);

    components = sphere.getPrincipalComponents(sphere._reachableDirections);

    // test phi
    ASSERT_TRUE(atan2(components[2].y, components[2].x) * 180.0 / M_PI < 72.0);
    ASSERT_TRUE(atan2(components[2].y, components[2].x) * 180.0 / M_PI > 68.0);

    // test theta
    ASSERT_TRUE(acos(components[2].z) * 180.0 / M_PI < 56.0);
    ASSERT_TRUE(acos(components[2].z) * 180.0 / M_PI > 54.0);


    // test Capability of type cylinder_2
    Capability cylinder2(CYLINDER_2, 70.0, 55.0, 35.0);
    
    sphere = createReachabilitySphereFromCapability(cylinder2, 400);

    components = sphere.getPrincipalComponents(sphere._reachableDirections);

    // test phi
    ASSERT_TRUE(atan2(components[0].y, components[0].x) * 180.0 / M_PI < 72.0);
    ASSERT_TRUE(atan2(components[0].y, components[0].x) * 180.0 / M_PI > 68.0);

    // test theta
    ASSERT_TRUE(acos(components[0].z) * 180.0 / M_PI < 56.0);
    ASSERT_TRUE(acos(components[0].z) * 180.0 / M_PI > 54.0);
}

TEST(ReachabilitySphere, fitCone)
{
    Capability cone1(CONE, 90.0, 90.0, 20.0);
    
    ReachabilitySphere sphere = createReachabilitySphereFromCapability(cone1, 200);
    
    // convert phi and theta to coordinates
    double phiInRad = 90.0 * M_PI / 180.0;
    double thetaInRad = 90.0 * M_PI / 180.0;

    Vector axis;
    axis.x = sin(thetaInRad) * cos(phiInRad);
    axis.y = sin(thetaInRad) * sin(phiInRad);
    axis.z = cos(thetaInRad);
    
    std::pair<double, double> angleAndSFE = sphere.fitCone(axis);
    ASSERT_TRUE(angleAndSFE.first < 20.01 && angleAndSFE.first > 19.0);
    ASSERT_EQ(angleAndSFE.second, 0.0);


    Capability cone2(CONE, 320.0, 35.0, 40.0);
    
    sphere = createReachabilitySphereFromCapability(cone2, 200);
    
    // convert phi and theta to coordinates
    phiInRad = 320.0 * M_PI / 180.0;
    thetaInRad = 35.0 * M_PI / 180.0;

    axis.x = sin(thetaInRad) * cos(phiInRad);
    axis.y = sin(thetaInRad) * sin(phiInRad);
    axis.z = cos(thetaInRad);
    
    angleAndSFE = sphere.fitCone(axis);
    ASSERT_TRUE(angleAndSFE.first < 40.01 && angleAndSFE.first > 39.0);
    ASSERT_EQ(angleAndSFE.second, 0.0);


    // check that no cone is matched if axis points to opposite direction
    Capability cone3(CONE, 0.0, 0.0, 20.0);
    
    sphere = createReachabilitySphereFromCapability(cone3, 200);
    
    // convert phi and theta to coordinates
    phiInRad = 0.0 * M_PI / 180.0;
    thetaInRad = 180.0 * M_PI / 180.0;

    axis.x = sin(thetaInRad) * cos(phiInRad);
    axis.y = sin(thetaInRad) * sin(phiInRad);
    axis.z = cos(thetaInRad);
    
    angleAndSFE = sphere.fitCone(axis);
    ASSERT_EQ(angleAndSFE.first, 0.0);
    ASSERT_EQ(angleAndSFE.second, 100.0);


    // check if SFE grows if axis is not alligned correctly
    Capability cone4(CONE, 0.0, 0.0, 20.0);
    
    sphere = createReachabilitySphereFromCapability(cone4, 200);
    
    // convert phi and theta to coordinates
    phiInRad = 0.0 * M_PI / 180.0;
    thetaInRad = 10.0 * M_PI / 180.0;

    axis.x = sin(thetaInRad) * cos(phiInRad);
    axis.y = sin(thetaInRad) * sin(phiInRad);
    axis.z = cos(thetaInRad);
    
    angleAndSFE = sphere.fitCone(axis);
    ASSERT_TRUE(angleAndSFE.second > 0.0);
}

TEST(ReachabilitySphere, fitCylinder_1)
{
    Capability cylinder1(CYLINDER_1, 90.0, 90.0, 20.0);
    
    ReachabilitySphere sphere = createReachabilitySphereFromCapability(cylinder1, 200);
    
    // convert phi and theta to coordinates
    double phiInRad = 90.0 * M_PI / 180.0;
    double thetaInRad = 90.0 * M_PI / 180.0;

    Vector axis;
    axis.x = sin(thetaInRad) * cos(phiInRad);
    axis.y = sin(thetaInRad) * sin(phiInRad);
    axis.z = cos(thetaInRad);
    
    std::pair<double, double> angleAndSFE = sphere.fitCylinder_1(axis);
    ASSERT_TRUE(angleAndSFE.first < 20.01 && angleAndSFE.first > 19.0);
    ASSERT_EQ(angleAndSFE.second, 0.0);


    Capability cylinder2(CYLINDER_1, 320.0, 35.0, 40.0);
    
    sphere = createReachabilitySphereFromCapability(cylinder2, 200);
    
    // convert phi and theta to coordinates
    phiInRad = 320.0 * M_PI / 180.0;
    thetaInRad = 35.0 * M_PI / 180.0;

    axis.x = sin(thetaInRad) * cos(phiInRad);
    axis.y = sin(thetaInRad) * sin(phiInRad);
    axis.z = cos(thetaInRad);
    
    angleAndSFE = sphere.fitCylinder_1(axis);
    ASSERT_TRUE(angleAndSFE.first < 40.01 && angleAndSFE.first > 39.0);
    ASSERT_EQ(angleAndSFE.second, 0.0);


    // check that cylinder_1 is matched too if axis points to opposite direction
    Capability cylinder3(CYLINDER_1, 0.0, 0.0, 20.0);
    
    sphere = createReachabilitySphereFromCapability(cylinder3, 200);
    
    // convert phi and theta to coordinates
    phiInRad = 0.0 * M_PI / 180.0;
    thetaInRad = 180.0 * M_PI / 180.0;

    axis.x = sin(thetaInRad) * cos(phiInRad);
    axis.y = sin(thetaInRad) * sin(phiInRad);
    axis.z = cos(thetaInRad);
    
    angleAndSFE = sphere.fitCylinder_1(axis);
    ASSERT_TRUE(angleAndSFE.first < 20.0);
    ASSERT_EQ(angleAndSFE.second, 0.0);


    // check if SFE grows if axis is not alligned correctly
    Capability cylinder4(CYLINDER_1, 0.0, 0.0, 20.0);
    
    sphere = createReachabilitySphereFromCapability(cylinder4, 200);
    
    // convert phi and theta to coordinates
    phiInRad = 0.0 * M_PI / 180.0;
    thetaInRad = 10.0 * M_PI / 180.0;

    axis.x = sin(thetaInRad) * cos(phiInRad);
    axis.y = sin(thetaInRad) * sin(phiInRad);
    axis.z = cos(thetaInRad);
    
    angleAndSFE = sphere.fitCylinder_1(axis);
    ASSERT_TRUE(angleAndSFE.second > 0.0);
}

TEST(ReachabilitySphere, fitCylinder_2)
{
    Capability cylinder1(CYLINDER_2, 90.0, 90.0, 20.0);
    
    ReachabilitySphere sphere = createReachabilitySphereFromCapability(cylinder1, 200);
    
    // convert phi and theta to coordinates
    double phiInRad = 90.0 * M_PI / 180.0;
    double thetaInRad = 90.0 * M_PI / 180.0;

    Vector axis;
    axis.x = sin(thetaInRad) * cos(phiInRad);
    axis.y = sin(thetaInRad) * sin(phiInRad);
    axis.z = cos(thetaInRad);
    
    std::pair<double, double> angleAndSFE = sphere.fitCylinder_2(axis);
    ASSERT_TRUE(angleAndSFE.first < 20.01);
    ASSERT_TRUE(angleAndSFE.first > 19.0);
    ASSERT_EQ(angleAndSFE.second, 0.0);


    Capability cylinder2(CYLINDER_2, 320.0, 35.0, 40.0);
    
    sphere = createReachabilitySphereFromCapability(cylinder2, 200);
    
    // convert phi and theta to coordinates
    phiInRad = 320.0 * M_PI / 180.0;
    thetaInRad = 35.0 * M_PI / 180.0;

    axis.x = sin(thetaInRad) * cos(phiInRad);
    axis.y = sin(thetaInRad) * sin(phiInRad);
    axis.z = cos(thetaInRad);
    
    angleAndSFE = sphere.fitCylinder_2(axis);
    ASSERT_TRUE(angleAndSFE.first < 40.01 && angleAndSFE.first > 39.0);
    ASSERT_EQ(angleAndSFE.second, 0.0);


    // check that cylinder_1 is matched too if axis points to opposite direction
    Capability cylinder3(CYLINDER_2, 0.0, 0.0, 20.0);
    
    sphere = createReachabilitySphereFromCapability(cylinder3, 200);
    
    // convert phi and theta to coordinates
    phiInRad = 0.0 * M_PI / 180.0;
    thetaInRad = 180.0 * M_PI / 180.0;

    axis.x = sin(thetaInRad) * cos(phiInRad);
    axis.y = sin(thetaInRad) * sin(phiInRad);
    axis.z = cos(thetaInRad);
    
    angleAndSFE = sphere.fitCylinder_2(axis);
    ASSERT_TRUE(angleAndSFE.first < 20.0);
    ASSERT_EQ(angleAndSFE.second, 0.0);


    // check if SFE grows if axis is not alligned correctly
    Capability cylinder4(CYLINDER_2, 0.0, 0.0, 20.0);
    
    sphere = createReachabilitySphereFromCapability(cylinder4, 200);
    
    // convert phi and theta to coordinates
    phiInRad = 0.0 * M_PI / 180.0;
    thetaInRad = 10.0 * M_PI / 180.0;

    axis.x = sin(thetaInRad) * cos(phiInRad);
    axis.y = sin(thetaInRad) * sin(phiInRad);
    axis.z = cos(thetaInRad);
    
    angleAndSFE = sphere.fitCylinder_2(axis);
    ASSERT_TRUE(angleAndSFE.second > 0.0);
}

TEST(ReachabilitySphere, convertToCapability)
{
    Capability capability(CONE, 70.0, 55.0, 25.0);
    
    ReachabilitySphere sphere = createReachabilitySphereFromCapability(capability, 500);

    Capability testCapability = sphere.convertToCapability();

    ASSERT_TRUE(testCapability.getType() == CONE);
    ASSERT_TRUE(std::abs(testCapability.getPhi() - capability.getPhi()) < 2.0);
    ASSERT_TRUE(std::abs(testCapability.getTheta() - capability.getTheta()) < 2.0);
    ASSERT_TRUE(std::abs(testCapability.getHalfOpeningAngle() - capability.getHalfOpeningAngle()) < 2.0);
    ASSERT_EQ(0.0, testCapability.getShapeFitError());


    capability = Capability(CYLINDER_1, 70.0, 55.0, 25.0);
    
    sphere = createReachabilitySphereFromCapability(capability, 10000);

    testCapability = sphere.convertToCapability();

    ASSERT_TRUE(testCapability.getType() == CYLINDER_1);
    ASSERT_TRUE(std::abs(testCapability.getPhi() - capability.getPhi()) < 2.0);
    ASSERT_TRUE(std::abs(testCapability.getTheta() - capability.getTheta()) < 2.0);
    ASSERT_TRUE(std::abs(testCapability.getHalfOpeningAngle() - capability.getHalfOpeningAngle()) < 2.0);
    // ASSERT_EQ(0.0, testCapability.getShapeFitError());
    // TODO: improve PCA to get correct axis (for now SFE could be 0.0 but is higher)


    capability = Capability(CYLINDER_2, 70.0, 55.0, 25.0);
    
    sphere = createReachabilitySphereFromCapability(capability, 10000);

    testCapability = sphere.convertToCapability();

    ASSERT_TRUE(testCapability.getType() == CYLINDER_2);
    ASSERT_TRUE(std::abs(testCapability.getPhi() - capability.getPhi()) < 2.0);
    ASSERT_TRUE(std::abs(testCapability.getTheta() - capability.getTheta()) < 2.0);
    ASSERT_TRUE(std::abs(testCapability.getHalfOpeningAngle() - capability.getHalfOpeningAngle()) < 2.0);
    // ASSERT_EQ(0.0, testCapability.getShapeFitError());
}

} // namespace

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}


