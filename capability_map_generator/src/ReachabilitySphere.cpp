#include "capability_map_generator/ReachabilitySphere.h"
#include "capability_map_generator/Vector.h"

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <cmath>
#include <algorithm>

namespace capability_map_generator
{

ReachabilitySphere::ReachabilitySphere()
{
    //ctor
}

ReachabilitySphere::~ReachabilitySphere()
{
    //dtor
}

void ReachabilitySphere::appendDirection(double phi, double theta, bool reachable)
{
    // convert phi and theta to coordinates
    double phiInRad = phi * M_PI / 180.0;
    double thetaInRad = theta * M_PI / 180.0;

    Vector newVector;
    newVector.x = sin(thetaInRad) * cos(phiInRad);
    newVector.y = sin(thetaInRad) * sin(phiInRad);
    newVector.z = cos(thetaInRad);

    if (reachable)
    {
        _reachableDirections.push_back(newVector);
    }
    else
    {
        _unreachableDirections.push_back(newVector);
    }
}

void ReachabilitySphere::appendDirection(double x, double y, double z, bool reachable)
{
    // normalize x, y and z
    double length = sqrt(x*x + y*y + z*z);
    x /= length;
    y /= length;
    z /= length;

    if (reachable)
    {
        _reachableDirections.push_back(Vector(x, y, z));
    }
    else
    {
        _unreachableDirections.push_back(Vector(x, y, z));
    }
}

void ReachabilitySphere::clear()
{
    _reachableDirections.clear();
    _unreachableDirections.clear();
}

Capability ReachabilitySphere::convertToCapability()
{
    // if there are no reachable directions return a empty capability
    if (_reachableDirections.size() == 0)
    {
        return Capability(EMPTY, 0.0, 0.0, 0.0);
    }
    // if there are no unreachable directions return a sphere capability
    if (_unreachableDirections.size() == 0)
    {
        return Capability(SPHERE, 0.0, 0.0, 0.0);
    }
    // having just one reachable direction causes the PCA to fail (division through zero), so catch this case
    if (_reachableDirections.size() == 1)
    {
        double phi = atan2(_reachableDirections[0].y, _reachableDirections[0].x) * 180.0 / M_PI;
        double theta = acos(_reachableDirections[0].z) * 180.0 / M_PI;
        // TODO: choose a good half opening angle in case there is just one direction reachable
        return Capability(CONE, phi, theta, 0.5, 0.0);
    }

    // TODO: current directions is only useful when filtering dataset later
    std::vector<Vector> currentDirections = _reachableDirections;

    // get principal components
    std::vector<Vector> eigenVectors = getPrincipalComponents(currentDirections);

    // try to fit different shapes to current directions (eigen sorts eigenvectors by increasing eigenvalues)

    // the eigenvector with the smallest eigenvalue is chosen to be the cone's axis
    std::pair<double, double> conePair = fitCone(eigenVectors[0]);

    // it is possible that the principal component points to the opposite direction, so try opposite direction too
    std::pair<double, double> conePair2 = fitCone(eigenVectors[0] * (-1.0));
    if (conePair2.second < conePair.second)
    {
        conePair = conePair2;
        eigenVectors[0] = eigenVectors[0] * (-1.0);
    }

    // the eigenvector with the greatest eigenvalue is chosen to be the cylinder_1's axis
    std::pair<double, double> cylinder_1Pair = fitCylinder_1(eigenVectors[2]);

    // the eigenvector with the smallest eigenvalue is chosen to be the cylinder_2's axis
    std::pair<double, double> cylinder_2Pair = fitCylinder_2(eigenVectors[0]);

    double sphereSFE = 100.0 * (double)_unreachableDirections.size() / (double)_reachableDirections.size();
    sphereSFE = sphereSFE > 100.0 ? 100.0 : sphereSFE;

    Capability retCapability;

    // select the shape type with smallest shapeFitError and create a capability (with equal SFE prefer cone)
    if (conePair.second <= cylinder_1Pair.second && conePair.second <= cylinder_2Pair.second)
    {
        // cone gets chosen
        double phi = atan2(eigenVectors[0].y, eigenVectors[0].x) * 180.0 / M_PI;
        double theta = acos(eigenVectors[0].z) * 180.0 / M_PI;

        retCapability = Capability(CONE, phi, theta, conePair.first, conePair.second);
    }
    else if (cylinder_1Pair.second < cylinder_2Pair.second)
    {
        // cylinder_1 gets chosen
        double phi = atan2(eigenVectors[2].y, eigenVectors[2].x) * 180.0 / M_PI;
        double theta = acos(eigenVectors[2].z) * 180.0 / M_PI;

        retCapability = Capability(CYLINDER_1, phi, theta, cylinder_1Pair.first, cylinder_1Pair.second);
    }
    else
    {
        // cylinder_2 gets chosen
        double phi = atan2(eigenVectors[0].y, eigenVectors[0].x) * 180.0 / M_PI;
        double theta = acos(eigenVectors[0].z) * 180.0 / M_PI;

        retCapability = Capability(CYLINDER_2, phi, theta, cylinder_2Pair.first, cylinder_2Pair.second);
    }

    // maybe a sphere fits better
    if (retCapability.getShapeFitError() >= sphereSFE)
    {
        retCapability = Capability(SPHERE, 0.0, 0.0, 0.0, sphereSFE);
    }
    return retCapability;
}


std::vector<Vector> ReachabilitySphere::getPrincipalComponents(const std::vector<Vector> &coords) const
{
    size_t size = coords.size();

    double meanX = 0.0;
    double meanY = 0.0;
    double meanZ = 0.0;

    // compute the mean
    for (size_t i = 0; i < size; ++i)
    {
        meanX += coords[i].x;
        meanY += coords[i].y;
        meanZ += coords[i].z;
    }
    meanX /= size;
    meanY /= size;
    meanZ /= size;

    double covXX = 0.0;
    double covYY = 0.0;
    double covZZ = 0.0;
    double covXY = 0.0;
    double covXZ = 0.0;
    double covYZ = 0.0;

    // compute covariances
    for (size_t i = 0; i < size; ++i)
    {
        covXX += (coords[i].x - meanX) * (coords[i].x - meanX);
        covYY += (coords[i].y - meanY) * (coords[i].y - meanY);
        covZZ += (coords[i].z - meanZ) * (coords[i].z - meanZ);
        covXY += (coords[i].x - meanX) * (coords[i].y - meanY);
        covXZ += (coords[i].x - meanX) * (coords[i].z - meanZ);
        covYZ += (coords[i].y - meanY) * (coords[i].z - meanZ);
    }
    covXX /= size - 1;
    covYY /= size - 1;
    covZZ /= size - 1;
    covXY /= size - 1;
    covXZ /= size - 1;
    covYZ /= size - 1;

    // compute eigenvalues and eigenvectors using the covariance matrix
    Eigen::Matrix3d covMatrix;
    covMatrix << covXX, covXY, covXZ,
                 covXY, covYY, covYZ,
                 covXZ, covYZ, covZZ;

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(covMatrix);

    Eigen::Matrix3d eigenVecs = es.eigenvectors();

    std::vector<Vector> eigenVectors(3);

    for (int i = 0; i < 3; ++i)
    {
        Vector vec;
        vec.x = eigenVecs(0, i);
        vec.y = eigenVecs(1, i);
        vec.z = eigenVecs(2, i);
        eigenVectors[i] = vec;
    }

    return eigenVectors;
}


std::pair<double, double> ReachabilitySphere::fitCone(const Vector &axis) const
{
    size_t numReachableDirections = _reachableDirections.size();
    size_t numUnreachableDirections = _unreachableDirections.size();

    double phiInRadAxis = atan2(axis.y, axis.x);
    double thetaInRadAxis = M_PI / 2.0 - acos(axis.z);

    // compute all possible orthodromeAngles and store them in a std::vector
    std::vector<double> reachableOrthodromeAngles(numReachableDirections);

    for (size_t i = 0; i < numReachableDirections; ++i)
    {
        // convert angles to rad
        double phiInRad = atan2(_reachableDirections[i].y, _reachableDirections[i].x);
        double thetaInRad = M_PI / 2.0 - acos(_reachableDirections[i].z);
        // calculate the shortest angle in rad (great-circle distance) between axis and current direction
        reachableOrthodromeAngles[i] = acos(sin(thetaInRadAxis) * sin(thetaInRad) + cos(thetaInRadAxis) *
                                            cos(thetaInRad) * cos(phiInRad - phiInRadAxis));
    }

    // same for unreachable directions
    std::vector<double> unreachableOrthodromeAngles(numUnreachableDirections);

    for (size_t i = 0; i < numUnreachableDirections; ++i)
    {
        // convert angles to rad
        double phiInRad = atan2(_unreachableDirections[i].y, _unreachableDirections[i].x);
        double thetaInRad = M_PI / 2.0 - acos(_unreachableDirections[i].z);
        // calculate the shortest angle in rad (great-circle distance) between axis and current direction
        unreachableOrthodromeAngles[i] = acos(sin(thetaInRadAxis) * sin(thetaInRad) + cos(thetaInRadAxis) *
                                              cos(thetaInRad) * cos(phiInRad - phiInRadAxis));
    }

    // sort the reachable and unreachable orthodrome angles (smallest first)
    std::sort(reachableOrthodromeAngles.begin(), reachableOrthodromeAngles.end());
    std::sort(unreachableOrthodromeAngles.begin(), unreachableOrthodromeAngles.end());

    size_t i = 0;
    size_t j = 0;
    int reachableMissed = numReachableDirections;
    int unreachableCovered = 0;
    double bestShapeFitError = 1.0;
    double bestAngle = 0.0;

    // compute angle with best shape fit error
    while (i < numReachableDirections && j < numUnreachableDirections)
    {
        // angle of current reachable direction is smaller than angle of current unreachable direction
        if (reachableOrthodromeAngles[i] < unreachableOrthodromeAngles[j])
        {
            // get next reachable angle and decrease reachableMissed
            ++i;
            --reachableMissed;
        }
        // a unreachable direction gets covered
        else
        {
            ++j;
            ++unreachableCovered;
        }
        // compute shape fit error and compare it with best shape fit error so far
        double shapeFitError = (double)(reachableMissed + unreachableCovered) / (double)numReachableDirections;

        if (shapeFitError <= bestShapeFitError)
        {
            bestShapeFitError = shapeFitError;
            bestAngle = reachableOrthodromeAngles[i - 1];
        }
    }

    bestShapeFitError *= 100.0;
    if (bestShapeFitError > 100.0)
    {
        bestShapeFitError = 100.0;
    }

    return std::make_pair(bestAngle * 180.0 / M_PI, bestShapeFitError);
}


std::pair<double, double> ReachabilitySphere::fitCylinder_1(const Vector &axis) const
{
    size_t numReachableDirections = _reachableDirections.size();
    size_t numUnreachableDirections = _unreachableDirections.size();

    double phiInRadAxis = atan2(axis.y, axis.x);
    double thetaInRadAxis = M_PI / 2.0 - acos(axis.z);

    // compute all possible orthodromeAngles and store them in a std::vector
    std::vector<double> reachableOrthodromeAngles(numReachableDirections);

    for (size_t i = 0; i < numReachableDirections; ++i)
    {
        // convert angles to rad
        double phiInRad = atan2(_reachableDirections[i].y, _reachableDirections[i].x);
        double thetaInRad = M_PI / 2.0 - acos(_reachableDirections[i].z);
        // calculate the shortest angle in rad (great-circle distance) between axis and current direction
        double tempAngle = acos(sin(thetaInRadAxis) * sin(thetaInRad) + cos(thetaInRadAxis) *
                                cos(thetaInRad) * cos(phiInRad - phiInRadAxis));
        // since cylinder_1 type is like a double cone, choose the smaller angle of orthodrome or orthodrome - 180°
        // which lies nearer to the opposite double cone side
        reachableOrthodromeAngles[i] = std::abs(tempAngle - M_PI) < tempAngle ? std::abs(tempAngle - M_PI) : tempAngle;
    }

    // same for unreachable directions
    std::vector<double> unreachableOrthodromeAngles(numUnreachableDirections);

    for (size_t i = 0; i < numUnreachableDirections; ++i)
    {
        // convert angles to rad
        double phiInRad = atan2(_unreachableDirections[i].y, _unreachableDirections[i].x);
        double thetaInRad = M_PI / 2.0 - acos(_unreachableDirections[i].z);
        // calculate the shortest angle in rad (great-circle distance) between axis and current direction
        double tempAngle = acos(sin(thetaInRadAxis) * sin(thetaInRad) + cos(thetaInRadAxis) *
                                cos(thetaInRad) * cos(phiInRad - phiInRadAxis));
        // since cylinder_1 type is like a double cone, choose the smaller angle of orthodrome or orthodrome - 180°
        // which lies nearer to the opposite double cone side
        unreachableOrthodromeAngles[i] = std::abs(tempAngle - M_PI) < tempAngle ? std::abs(tempAngle - M_PI) : tempAngle;
    }

    // sort the reachable and unreachable orthodrome angles (smallest first)
    std::sort(reachableOrthodromeAngles.begin(), reachableOrthodromeAngles.end());
    std::sort(unreachableOrthodromeAngles.begin(), unreachableOrthodromeAngles.end());

    size_t i = 0;
    size_t j = 0;
    int reachableMissed = numReachableDirections;
    int unreachableCovered = 0;
    double bestShapeFitError = 1.0;
    double bestAngle = 0.0;

    // compute angle with best shape fit error
    while (i < numReachableDirections && j < numUnreachableDirections)
    {
        // angle of current reachable direction is smaller than angle of current unreachable direction
        if (reachableOrthodromeAngles[i] < unreachableOrthodromeAngles[j])
        {
            // get next reachable angle and decrease reachableMissed
            ++i;
            --reachableMissed;
        }
        // a unreachable direction gets covered
        else
        {
            ++j;
            ++unreachableCovered;
        }
        // compute shape fit error and compare it with best shape fit error so far
        double shapeFitError = (double)(reachableMissed + unreachableCovered) / (double)numReachableDirections;

        if (shapeFitError <= bestShapeFitError)
        {
            bestShapeFitError = shapeFitError;
            bestAngle = reachableOrthodromeAngles[i - 1];
        }
    }

    // In the case there are just one or two unreachable directions not covered, a cylinder_1 type is not a good decision.
    // It is possible that they lie on an orthodrome which doesn't cover any other point. A cone or a sphere would be more accurate
    if (numUnreachableDirections - unreachableCovered < 3)
    {
        bestShapeFitError = 1.0;
    }

    bestShapeFitError *= 100.0;
    if (bestShapeFitError > 100.0)
    {
        bestShapeFitError = 100.0;
    }

    return std::make_pair(bestAngle * 180.0 / M_PI, bestShapeFitError);
}


std::pair<double, double> ReachabilitySphere::fitCylinder_2(const Vector &axis) const
{
    size_t numReachableDirections = _reachableDirections.size();
    size_t numUnreachableDirections = _unreachableDirections.size();

    double phiInRadAxis = atan2(axis.y, axis.x);
    double thetaInRadAxis = M_PI / 2.0 - acos(axis.z);

    // compute all possible orthodromeAngles and store them in a std::vector
    std::vector<double> reachableOrthodromeAngles(numReachableDirections);

    for (size_t i = 0; i < numReachableDirections; ++i)
    {
        // convert angles to rad
        double phiInRad = atan2(_reachableDirections[i].y, _reachableDirections[i].x);
        double thetaInRad = M_PI / 2.0 - acos(_reachableDirections[i].z);
        // calculate the shortest angle in rad (great-circle distance) between axis and current direction
        // since the sensitive part of cylinder_2 is at 90° around the axis subtract the orthodrome from PI/2
        reachableOrthodromeAngles[i] = std::abs(M_PI / 2.0 - acos(sin(thetaInRadAxis) * sin(thetaInRad) + cos(thetaInRadAxis) *
                                                                  cos(thetaInRad) * cos(phiInRad - phiInRadAxis)));
    }

    // same for unreachable directions
    std::vector<double> unreachableOrthodromeAngles(numUnreachableDirections);

    for (size_t i = 0; i < numUnreachableDirections; ++i)
    {
        // convert angles to rad
        double phiInRad = atan2(_unreachableDirections[i].y, _unreachableDirections[i].x);
        double thetaInRad = M_PI / 2.0 - acos(_unreachableDirections[i].z);
        // calculate the shortest angle in rad (great-circle distance) between axis and current direction
        // since the sensitive part of cylinder_2 is at 90° around the axis subtract the orthodrome from PI/2
        unreachableOrthodromeAngles[i] = std::abs(M_PI / 2.0 - acos(sin(thetaInRadAxis) * sin(thetaInRad) + cos(thetaInRadAxis) *
                                                                    cos(thetaInRad) * cos(phiInRad - phiInRadAxis)));
    }

    // sort the reachable and unreachable orthodrome angles (smallest first)
    std::sort(reachableOrthodromeAngles.begin(), reachableOrthodromeAngles.end());
    std::sort(unreachableOrthodromeAngles.begin(), unreachableOrthodromeAngles.end());

    size_t i = 0;
    size_t j = 0;
    int reachableMissed = numReachableDirections;
    int unreachableCovered = 0;
    double bestShapeFitError = 1.0;
    double bestAngle = 0.0;

    // compute angle with best shape fit error
    while (i < numReachableDirections && j < numUnreachableDirections)
    {
        // angle of current reachable direction is smaller than angle of current unreachable direction
        if (reachableOrthodromeAngles[i] < unreachableOrthodromeAngles[j])
        {
            // get next reachable angle and decrease reachableMissed
            ++i;
            --reachableMissed;
        }
        // a unreachable direction gets covered
        else
        {
            ++j;
            ++unreachableCovered;
        }
        // compute shape fit error and compare it with best shape fit error so far
        double shapeFitError = (double)(reachableMissed + unreachableCovered) / (double)numReachableDirections;

        if (shapeFitError <= bestShapeFitError)
        {
            bestShapeFitError = shapeFitError;
            bestAngle = reachableOrthodromeAngles[i - 1];
        }
    }

    // In the case there are just one or two reachable directions covered, a cylinder_2 type is not a good decision.
    // It is possible that they lie on an orthodrome which doesn't cover any other point.
    if (numReachableDirections - reachableMissed < 3)
    {
        bestShapeFitError = 1.0;
    }

    bestShapeFitError *= 100.0;
    if (bestShapeFitError > 100.0)
    {
        bestShapeFitError = 100.0;
    }

    return std::make_pair(bestAngle * 180.0 / M_PI, bestShapeFitError);
}

} // namespace

