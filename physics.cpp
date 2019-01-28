// Headers
#include "jello.h"
#include "physics.h"
#include <string>
#include <iostream>
#include <vector>

/**
 * calcCollisionDampForce - Calculates the Damping Force for a Collision
 *                          on a Mass Point
 */
struct point calcCollisionDampForce(struct world *jello, int i, int j, int k, struct point l)
{
    // Initialize Damp Force
    point dampForce;

    // Calculate the Projection ((vA-vB) dot L)
    double projection;
    DOTPRODUCTp(jello->v[i][j][k], l, projection);

    // Get the Magnitude of the Length
    double mag;
    pMAG(l, mag);

    // Normalize the Projection ((vA-vB) dot L)/|L|)
    double normProjection = projection / mag;

    // Multiply by the Damping Coefficieint
    double kLength = jello->dCollision * normProjection;

    // Normalize the Length
    double length;
    pNORMALIZE(l);

    // Calculate the Force F = -kCollisionDamp(((vA-vB) dot L)/|L|)(L/|L|)
    pMULTIPLY(l, kLength, dampForce);

    return dampForce;
}

/**
 * calcCollisionHookForce - Calculates Hook's Law for a Collision
 *                          on a given Mass Point
 */
struct point calcCollisionHookForce(struct world *jello, struct point l, double rLength)
{
    // Initialize Hook Force
    point hookForce;

    // Get the Magnitude of the Length
    double mag;
    pMAG(l, mag);

    // Calculate the Length (|L| - R)
    double dX = mag - rLength;

    // Multiply by Hook's Constant -kHook(|L| - R)
    double kLength = jello->kCollision * dX;

    // Normalize the Length
    double length;
    pNORMALIZE(l);

    // Calculate the Force F = kCollisionHook(|L| - R)(L/|L|)
    pMULTIPLY(l, kLength, hookForce);

    return hookForce;
}

/**
 * calcDampForce - Calculates the Damping Force
 *                 on a Mass Point
 */
struct point calcDampForce(int i, int j, int k, struct world *jello, struct point l, struct point vDiff)
{
    // Initialize Damp Force
    point dampForce;

    // Calculate the Projection ((vA-vB) dot L)
    double projection;
    DOTPRODUCTp(vDiff, l, projection);

    // Get the Magnitude of the Length
    double mag;
    pMAG(l, mag);

    // Normalize the Projection ((vA-vB) dot L)/|L|)
    double normProjection = projection / mag;

    // Multiply by the Damping Coefficieint
    double kLength = -1 * jello->dElastic * normProjection;

    // Ensure Numerics aren't zero to prevent divide by zero case
    if((jello->p[i][j][k].x != 0.0) && (jello->p[i][j][k].y != 0.0) && (jello->p[i][j][k].z != 0.0))
    {
        // Normalize the Length
        double length;
        pNORMALIZE(l);
    }

    // Calculate the Force F = -kDamp(((vA-vB) dot L)/|L|)(L/|L|)
    pMULTIPLY(l, kLength, dampForce);

    return dampForce;
}

/**
 * calcHookForce - Calculates Hook's Law
 *                 on a given Mass Point
 *                 and returns the Force
 *
 */
struct point calcHookForce(int i, int j, int k, struct world *jello, struct point l, double rLength)
{
    // Initialize Hook Force
    point hookForce;

    // Get the Magnitude of the Length
    double mag;
    pMAG(l, mag);

    // Calculate the Length (|L| - R)
    double dX = mag - rLength;

    // Multiply by Hook's Constant -kHook(|L| - R)
    double kLength = -1 * jello->kElastic * dX;

    // Ensure Numerics aren't zero to prevent divide by zero case
    if((jello->p[i][j][k].x != 0.0) && (jello->p[i][j][k].y != 0.0) && (jello->p[i][j][k].z != 0.0))
    {
        // Normalize the Length
        double length;
        pNORMALIZE(l);
    }

    // Calculate the Force F = -kHook(|L| - R)(L/|L|)
    pMULTIPLY(l, kLength, hookForce);

    return hookForce;
}

/**
 * checkCollision - Checks if a Collision has Occurred
 */
bool checkCollision(int i, int j, int k, struct world *jello)
{
   //Initialize Collision Indicator
   bool isCollision = false;

   // Check if Mass Point has Left Bounding Box
   if((jello->p[i][j][k].x <= -2.0) || (jello->p[i][j][k].x >= 2.0) ||
      (jello->p[i][j][k].y <= -2.0) || (jello->p[i][j][k].y >= 2.0) ||
      (jello->p[i][j][k].z <= -2.0) || (jello->p[i][j][k].z >= 2.0))
   {
       isCollision = true;
   }

   return isCollision;
}

/**
 * calcExternalForce - Calculates the Force exerted
 *                     by an External Force Field
 *                     acting on the Mass Point
 */
struct point calcExternalForce(int i, int j, int k, struct world *jello)
{
    //Initialize External Force
    point extForce;

//    // Get Resolution
//    int resolution = jello->resolution;
//
//    // Check if there is a Force Field
//    if(resolution != 0)
//    {
//        // Initialize Diff X / Diff Y / Diff Z
//        double diffX;
//        double diffY;
//        double diffZ;
//
//        // Get the 8 Vertices of the Force Field Cube
//        point c000 = jello->forceField[resolution, resolution, resolution];
//        point c001 = jello->forceField[resolution, resolution, resolution];
//        point c101 = jello->forceField[resolution, resolution, resolution];
//        point c100 = jello->forceField[resolution, resolution, resolution];
//        point c010 = jello->forceField[resolution, resolution, resolution];
//        point c110 = jello->forceField[resolution, resolution, resolution];
//        point c011 = jello->forceField[resolution, resolution, resolution];
//        point c111 = jello->forceField[resolution, resolution, resolution];
//
//        // Calculate External Force (Trilinear Interpolation)
//        extForce.x = (((1 - diffX) * (1 - diffY) * (1 - diffZ)) * c000.x +
//                      ((1 - diffX) * (1 - diffY) * (diffZ)) * c001.x +
//                      ((diffX) * (1 - diffY) * (diffZ)) * c101.x +
//                      ((diffX) * (1 - diffY) * (1 - diffZ)) * c100.x +
//                      ((1 - diffX) * (diffY) * (1 - diffZ)) * c010.x +
//                      ((diffX) * (diffY) * (1 - diffZ)) * c110.x +
//                      ((diffX) * (diffY) * (diffZ)) * c111.x +
//                      ((1 - diffX) * (diffY) * (diffZ)) * c011.x);
//
//        extForce.y = (((1 - diffX) * (1 - diffY) * (1 - diffZ)) * c000.y +
//                      ((1 - diffX) * (1 - diffY) * (diffZ)) * c001.y +
//                      ((diffX) * (1 - diffY) * (diffZ)) * c101.y +
//                      ((diffX) * (1 - diffY) * (1 - diffZ)) * c100.y +
//                      ((1 - diffX) * (diffY) * (1 - diffZ)) * c010.y +
//                      ((diffX) * (diffY) * (1 - diffZ)) * c110.y +
//                      ((diffX) * (diffY) * (diffZ)) * c111.y +
//                      ((1 - diffX) * (diffY) * (diffZ)) * c011.y);
//
//        extForce.z = (((1 - diffX) * (1 - diffY) * (1 - diffZ)) * c000.z +
//                      ((1 - diffX) * (1 - diffY) * (diffZ)) * c001.z +
//                      ((diffX) * (1 - diffY) * (diffZ)) * c101.z +
//                      ((diffX) * (1 - diffY) * (1 - diffZ)) * c100.z +
//                      ((1 - diffX) * (diffY) * (1 - diffZ)) * c010.z +
//                      ((diffX) * (diffY) * (1 - diffZ)) * c110.z +
//                      ((diffX) * (diffY) * (diffZ)) * c111.z +
//                      ((1 - diffX) * (diffY) * (diffZ)) * c011.z);
//    }

    return extForce;
}

/**
 * processCollision - Process Penalty Force based on Collision
 *                    with the Bouding Box
 */
struct point processCollision(int i, int j, int k, struct world *jello)
{
    // Initialize Collision Force
    point collisionForce;

    // Initilaize Attributes
    point l;     // L Vector
    point vDiff; // Velocity Difference

    // Initialize Rest Length (0 for a Collision Spring)
    double restLength = 0;

    // Check Out of Bounds Case 1 (X Min)
    if(jello->p[i][j][k].x <= -2.0)
    {
        // Make Bounding Point
        struct point xMinBound;
        xMinBound.x = -2.0;
        xMinBound.y =  0.0;
        xMinBound.z =  0.0;

        // Get Vector L (Vector pointing from B to A)
        pDIFFERENCE(jello->p[i][j][k], xMinBound, l);

        // Calculate Forces
        point hookForce = calcCollisionHookForce(jello, l, restLength); // Hook's Law
        point dampForce = calcCollisionDampForce(jello, i, j, k, l);    // Damping

        // Accumulate Forces
        pSUM(collisionForce, hookForce, collisionForce);
        pSUM(collisionForce, dampForce, collisionForce);
    }

    // Check Out of Bounds Case 2 (X Max)
    if(jello->p[i][j][k].x >= 2.0)
    {
        // Make Bounding Point
        struct point xMaxBound;
        xMaxBound.x = 2.0;
        xMaxBound.y = 0.0;
        xMaxBound.z = 0.0;

        // Get Vector L (Vector pointing from B to A)
        pDIFFERENCE(jello->p[i][j][k], xMaxBound, l);

        // Calculate Forces
        point hookForce = calcCollisionHookForce(jello, l, restLength); // Hook's Law
        point dampForce = calcCollisionDampForce(jello, i, j, k, l);    // Damping

        // Accumulate Forces
        pSUM(collisionForce, hookForce, collisionForce);
        pSUM(collisionForce, dampForce, collisionForce);
    }

    // Check Out of Bounds Case 3 (Y Min)
    if(jello->p[i][j][k].y <= -2.0)
    {
        // Make Bounding Point
        struct point yMinBound;
        yMinBound.x =  0.0;
        yMinBound.y = -2.0;
        yMinBound.z =  0.0;

        // Get Vector L (Vector pointing from B to A)
        pDIFFERENCE(jello->p[i][j][k], yMinBound, l);

        // Calculate Forces
        point hookForce = calcCollisionHookForce(jello, l, restLength); // Hook's Law
        point dampForce = calcCollisionDampForce(jello, i, j, k, l);    // Damping

        // Accumulate Forces
        pSUM(collisionForce, hookForce, collisionForce);
        pSUM(collisionForce, dampForce, collisionForce);
    }

    // Check Out of Bounds Case 4 (Y Max)
    if(jello->p[i][j][k].y >= 2.0)
    {
        // Make Bounding Point
        struct point yMaxBound;
        yMaxBound.x = 0.0;
        yMaxBound.y = 2.0;
        yMaxBound.z = 0.0;

        // Get Vector L (Vector pointing from B to A)
        pDIFFERENCE(jello->p[i][j][k], yMaxBound, l);

        // Calculate Forces
        point hookForce = calcCollisionHookForce(jello, l, restLength); // Hook's Law
        point dampForce = calcCollisionDampForce(jello, i, j, k, l);    // Damping

        // Accumulate Forces
        pSUM(collisionForce, hookForce, collisionForce);
        pSUM(collisionForce, dampForce, collisionForce);
    }

    // Check Out of Bounds Case 5 (Z Min)
    if(jello->p[i][j][k].z <= -2.0)
    {
        // Make Bounding Point
        struct point zMinBound;
        zMinBound.x =  0.0;
        zMinBound.y =  0.0;
        zMinBound.z = -2.0;

        // Get Vector L (Vector pointing from B to A)
        pDIFFERENCE(jello->p[i][j][k], zMinBound, l);

        // Calculate Forces
        point hookForce = calcCollisionHookForce(jello, l, restLength); // Hook's Law
        point dampForce = calcCollisionDampForce(jello, i, j, k, l);    // Damping

        // Accumulate Forces
        pSUM(collisionForce, hookForce, collisionForce);
        pSUM(collisionForce, dampForce, collisionForce);
    }

    // Check Out of Bounds Case 6 (Z Max)
    if(jello->p[i][j][k].z >= 2.0)
    {
        // Make Bounding Point
        struct point zMaxBound;
        zMaxBound.x = 0.0;
        zMaxBound.y = 0.0;
        zMaxBound.z = 2.0;

        // Get Vector L (Vector pointing from B to A)
        pDIFFERENCE(jello->p[i][j][k], zMaxBound, l);

        // Calculate Forces
        point hookForce = calcCollisionHookForce(jello, l, restLength); // Hook's Law
        point dampForce = calcCollisionDampForce(jello, i, j, k, l);    // Damping

        // Accumulate Forces
        pSUM(collisionForce, hookForce, collisionForce);
        pSUM(collisionForce, dampForce, collisionForce);
    }

    return collisionForce;
}

/**
 * processStructSprings - Process Accumulation of Forces on Structural Springs
 */
struct point processStructSprings(int i, int j, int k, struct world *jello)
{
    // Initialize Struct Force
    point structForce;

    // Initilaize Attributes
    point l;     // L Vector
    point vDiff; // Velocity Difference

    // Initialize Rest Length
    double restLength = (1.0/7.0);

    // Initialize Neighbor Array
    std::vector<particle> neighbors;

    // Check Neighbor 1 Case
    if(i != 7)
    {
        // Create Mass Point
        particle massPoint;
        massPoint.position = jello->p[i+1][j][k];
        massPoint.velocity = jello->v[i+1][j][k];

        neighbors.push_back(massPoint);
    }

    // Check Neighbor 2 Case
    if(i != 0)
    {
        // Get Vector L (Vector pointing from B to A)
        pDIFFERENCE(jello->p[i][j][k], jello->p[i-1][j][k], l);

        // Get Difference in Velocities of points A and B
        pDIFFERENCE(jello->v[i][j][k], jello->v[i-1][j][k], vDiff);

        // Calculate Forces
        point hookForce = calcHookForce(i, j, k, jello, l, restLength); // Hook's Law
        point dampForce = calcDampForce(i, j, k, jello, l, vDiff);      // Damping

        // Accumulate Forces
        pSUM(structForce, hookForce, structForce);
        pSUM(structForce, dampForce, structForce);
    }

    // Check Neighbor 3 Case
    if(j != 7)
    {
        // Get Vector L (Vector pointing from B to A)
        pDIFFERENCE(jello->p[i][j][k], jello->p[i][j+1][k], l);

        // Get Difference in Velocities of points A and B
        pDIFFERENCE(jello->v[i][j][k], jello->v[i][j+1][k], vDiff);

        // Calculate Forces
        point hookForce = calcHookForce(i, j, k, jello, l, restLength); // Hook's Law
        point dampForce = calcDampForce(i, j, k, jello, l, vDiff);      // Damping

        // Accumulate Forces
        pSUM(structForce, hookForce, structForce);
        pSUM(structForce, dampForce, structForce);
    }

    // Check Neighbor 4 Case
    if(j != 0)
    {
        // Get Vector L (Vector pointing from B to A)
        pDIFFERENCE(jello->p[i][j][k], jello->p[i][j-1][k], l);

        // Get Difference in Velocities of points A and B
        pDIFFERENCE(jello->v[i][j][k], jello->v[i][j-1][k], vDiff);

        // Calculate Forces
        point hookForce = calcHookForce(i, j, k, jello, l, restLength); // Hook's Law
        point dampForce = calcDampForce(i, j, k, jello, l, vDiff);      // Damping

        // Accumulate Forces
        pSUM(structForce, hookForce, structForce);
        pSUM(structForce, dampForce, structForce);
    }

    // Check Neighbor 5 Case
    if(k != 7)
    {
        // Get Vector L (Vector pointing from B to A)
        pDIFFERENCE(jello->p[i][j][k], jello->p[i][j][k+1], l);

        // Get Difference in Velocities of points A and B
        pDIFFERENCE(jello->v[i][j][k], jello->v[i][j][k+1], vDiff);

        // Calculate Forces
        point hookForce = calcHookForce(i, j, k, jello, l, restLength); // Hook's Law
        point dampForce = calcDampForce(i, j, k, jello, l, vDiff);      // Damping

        // Accumulate Forces
        pSUM(structForce, hookForce, structForce);
        pSUM(structForce, dampForce, structForce);
    }

    // Check Neighbor 6 Case
    if(k != 0)
    {
        // Get Vector L (Vector pointing from B to A)
        pDIFFERENCE(jello->p[i][j][k], jello->p[i][j][k-1], l);

        // Get Difference in Velocities of points A and B
        pDIFFERENCE(jello->v[i][j][k], jello->v[i][j][k-1], vDiff);

        // Calculate Forces
        point hookForce = calcHookForce(i, j, k, jello, l, restLength); // Hook's Law
        point dampForce = calcDampForce(i, j, k, jello, l, vDiff);      // Damping

        // Accumulate Forces
        pSUM(structForce, hookForce, structForce);
        pSUM(structForce, dampForce, structForce);
    }

    // Iterate over the Neighbor Positions
    for(auto &neighbor : neighbors)
    {
        // Get Vector L (Vector pointing from B to A)
        pDIFFERENCE(jello->p[i][j][k], neighbor.position, l);

        // Get Difference in Velocities of points A and B
        pDIFFERENCE(jello->v[i][j][k], neighbor.velocity, vDiff);

        // Calculate Forces
        point hookForce = calcHookForce(i, j, k, jello, l, restLength); // Hook's Law
        point dampForce = calcDampForce(i, j, k, jello, l, vDiff);      // Damping

        // Accumulate Forces
        pSUM(structForce, hookForce, structForce);
        pSUM(structForce, dampForce, structForce);
    }

    return structForce;
}

/**
 * processShearSprings - Computes all the Forces acting
 *                       on the Shear Springs
 */
struct point processShearSprings(int i, int j, int k, struct world *jello)
{
    // Initialize Shear Force
    point shearForce;

    // Initilaize Attributes
    point l;     // L Vector
    point vDiff; // Velocity Difference

    // Initialize Rest Lengths
    double mainRestLength = (1.0/7.0) * (sqrt(3));
    double sideRestLength = (1.0/7.0) * (sqrt(2));

    // Side Diagonal Case 1
    if((i != 7) && (j != 7))
    {
        // Get Vector L (Vector pointing from B to A)
        pDIFFERENCE(jello->p[i][j][k], jello->p[i+1][j+1][k], l);

        // Get Difference in Velocities of points A and B
        pDIFFERENCE(jello->v[i][j][k], jello->v[i+1][j+1][k], vDiff);

        // Calculate Forces
        point hookForce = calcHookForce(i, j, k, jello, l, sideRestLength); // Hook's Law
        point dampForce = calcDampForce(i, j, k, jello, l, vDiff);          // Damping

        // Accumulate Forces
        pSUM(shearForce, hookForce, shearForce);
        pSUM(shearForce, dampForce, shearForce);
    }

    // Side Diagonal Case 2
    if((i != 0) && (j != 7))
    {
        // Get Vector L (Vector pointing from B to A)
        pDIFFERENCE(jello->p[i][j][k], jello->p[i-1][j+1][k], l);

        // Get Difference in Velocities of points A and B
        pDIFFERENCE(jello->v[i][j][k], jello->v[i-1][j+1][k], vDiff);

        // Calculate Forces
        point hookForce = calcHookForce(i, j, k, jello, l, sideRestLength); // Hook's Law
        point dampForce = calcDampForce(i, j, k, jello, l, vDiff);          // Damping

        // Accumulate Forces
        pSUM(shearForce, hookForce, shearForce);
        pSUM(shearForce, dampForce, shearForce);
    }

    // Side Diagonal Case 3
    if((i != 7) && (j != 0))
    {
        // Get Vector L (Vector pointing from B to A)
        pDIFFERENCE(jello->p[i][j][k], jello->p[i+1][j-1][k], l);

        // Get Difference in Velocities of points A and B
        pDIFFERENCE(jello->v[i][j][k], jello->v[i+1][j-1][k], vDiff);

        // Calculate Forces
        point hookForce = calcHookForce(i, j, k, jello, l, sideRestLength); // Hook's Law
        point dampForce = calcDampForce(i, j, k, jello, l, vDiff);          // Damping

        // Accumulate Forces
        pSUM(shearForce, hookForce, shearForce);
        pSUM(shearForce, dampForce, shearForce);
    }

    // Side Diagonal Case 4
    if((i != 0) && (j != 0))
    {
        // Get Vector L (Vector pointing from B to A)
        pDIFFERENCE(jello->p[i][j][k], jello->p[i-1][j-1][k], l);

        // Get Difference in Velocities of points A and B
        pDIFFERENCE(jello->v[i][j][k], jello->v[i-1][j-1][k], vDiff);

        // Calculate Forces
        point hookForce = calcHookForce(i, j, k, jello, l, sideRestLength); // Hook's Law
        point dampForce = calcDampForce(i, j, k, jello, l, vDiff);          // Damping

        // Accumulate Forces
        pSUM(shearForce, hookForce, shearForce);
        pSUM(shearForce, dampForce, shearForce);
    }

    // Side Diagonal Case 5
    if((j != 7) && (k != 7))
    {
        // Get Vector L (Vector pointing from B to A)
        pDIFFERENCE(jello->p[i][j][k], jello->p[i][j+1][k+1], l);

        // Get Difference in Velocities of points A and B
        pDIFFERENCE(jello->v[i][j][k], jello->v[i][j+1][k+1], vDiff);

        // Calculate Forces
        point hookForce = calcHookForce(i, j, k, jello, l, sideRestLength); // Hook's Law
        point dampForce = calcDampForce(i, j, k, jello, l, vDiff);          // Damping

        // Accumulate Forces
        pSUM(shearForce, hookForce, shearForce);
        pSUM(shearForce, dampForce, shearForce);
    }

    // Side Diagonal Case 6
    if((j != 0) && (k != 7))
    {
        // Get Vector L (Vector pointing from B to A)
        pDIFFERENCE(jello->p[i][j][k], jello->p[i][j-1][k+1], l);

        // Get Difference in Velocities of points A and B
        pDIFFERENCE(jello->v[i][j][k], jello->v[i][j-1][k+1], vDiff);

        // Calculate Forces
        point hookForce = calcHookForce(i, j, k, jello, l, sideRestLength); // Hook's Law
        point dampForce = calcDampForce(i, j, k, jello, l, vDiff);          // Damping

        // Accumulate Forces
        pSUM(shearForce, hookForce, shearForce);
        pSUM(shearForce, dampForce, shearForce);
    }

    // Side Diagonal Case 7
    if((j != 7) && (k != 0))
    {
        // Get Vector L (Vector pointing from B to A)
        pDIFFERENCE(jello->p[i][j][k], jello->p[i][j+1][k-1], l);

        // Get Difference in Velocities of points A and B
        pDIFFERENCE(jello->v[i][j][k], jello->v[i][j+1][k-1], vDiff);

        // Calculate Forces
        point hookForce = calcHookForce(i, j, k, jello, l, sideRestLength); // Hook's Law
        point dampForce = calcDampForce(i, j, k, jello, l, vDiff);          // Damping

        // Accumulate Forces
        pSUM(shearForce, hookForce, shearForce);
        pSUM(shearForce, dampForce, shearForce);
    }

    // Side Diagonal Case 8
    if((j != 0) && (k != 0))
    {
        // Get Vector L (Vector pointing from B to A)
        pDIFFERENCE(jello->p[i][j][k], jello->p[i][j-1][k-1], l);

        // Get Difference in Velocities of points A and B
        pDIFFERENCE(jello->v[i][j][k], jello->v[i][j-1][k-1], vDiff);

        // Calculate Forces
        point hookForce = calcHookForce(i, j, k, jello, l, sideRestLength); // Hook's Law
        point dampForce = calcDampForce(i, j, k, jello, l, vDiff);          // Damping

        // Accumulate Forces
        pSUM(shearForce, hookForce, shearForce);
        pSUM(shearForce, dampForce, shearForce);
    }

    // Side Diagonal Case 9
    if((i != 7) && (k != 7))
    {
        // Get Vector L (Vector pointing from B to A)
        pDIFFERENCE(jello->p[i][j][k], jello->p[i+1][j][k+1], l);

        // Get Difference in Velocities of points A and B
        pDIFFERENCE(jello->v[i][j][k], jello->v[i+1][j][k+1], vDiff);

        // Calculate Forces
        point hookForce = calcHookForce(i, j, k, jello, l, sideRestLength); // Hook's Law
        point dampForce = calcDampForce(i, j, k, jello, l, vDiff);          // Damping

        // Accumulate Forces
        pSUM(shearForce, hookForce, shearForce);
        pSUM(shearForce, dampForce, shearForce);
    }

    // Side Diagonal Case 10
    if((i != 0) && (k != 7))
    {
        // Get Vector L (Vector pointing from B to A)
        pDIFFERENCE(jello->p[i][j][k], jello->p[i-1][j][k+1], l);

        // Get Difference in Velocities of points A and B
        pDIFFERENCE(jello->v[i][j][k], jello->v[i-1][j][k+1], vDiff);

        // Calculate Forces
        point hookForce = calcHookForce(i, j, k, jello, l, sideRestLength); // Hook's Law
        point dampForce = calcDampForce(i, j, k, jello, l, vDiff);          // Damping

        // Accumulate Forces
        pSUM(shearForce, hookForce, shearForce);
        pSUM(shearForce, dampForce, shearForce);
    }

    // Side Diagonal Case 11
    if((i != 7) && (k != 0))
    {
        // Get Vector L (Vector pointing from B to A)
        pDIFFERENCE(jello->p[i][j][k], jello->p[i+1][j][k-1], l);

        // Get Difference in Velocities of points A and B
        pDIFFERENCE(jello->v[i][j][k], jello->v[i+1][j][k-1], vDiff);

        // Calculate Forces
        point hookForce = calcHookForce(i, j, k, jello, l, sideRestLength); // Hook's Law
        point dampForce = calcDampForce(i, j, k, jello, l, vDiff);          // Damping

        // Accumulate Forces
        pSUM(shearForce, hookForce, shearForce);
        pSUM(shearForce, dampForce, shearForce);
    }

    // Side Diagonal Case 12
    if((i != 0) && (k != 0))
    {
        // Get Vector L (Vector pointing from B to A)
        pDIFFERENCE(jello->p[i][j][k], jello->p[i-1][j][k-1], l);

        // Get Difference in Velocities of points A and B
        pDIFFERENCE(jello->v[i][j][k], jello->v[i-1][j][k-1], vDiff);

        // Calculate Forces
        point hookForce = calcHookForce(i, j, k, jello, l, sideRestLength); // Hook's Law
        point dampForce = calcDampForce(i, j, k, jello, l, vDiff);          // Damping

        // Accumulate Forces
        pSUM(shearForce, hookForce, shearForce);
        pSUM(shearForce, dampForce, shearForce);
    }

    // Main Diagonal Case 1
    if((i != 7) && (j != 7) && (k != 7))
    {
        // Get Vector L (Vector pointing from B to A)
        pDIFFERENCE(jello->p[i][j][k], jello->p[i+1][j+1][k+1], l);

        // Get Difference in Velocities of points A and B
        pDIFFERENCE(jello->v[i][j][k], jello->v[i+1][j+1][k+1], vDiff);

        // Calculate Forces
        point hookForce = calcHookForce(i, j, k, jello, l, mainRestLength); // Hook's Law
        point dampForce = calcDampForce(i, j, k, jello, l, vDiff);          // Damping

        // Accumulate Forces
        pSUM(shearForce, hookForce, shearForce);
        pSUM(shearForce, dampForce, shearForce);
    }

    // Main Diagonal Case 2
    if((i != 7) && (j != 7) && (k != 0))
    {
        // Get Vector L (Vector pointing from B to A)
        pDIFFERENCE(jello->p[i][j][k], jello->p[i+1][j+1][k-1], l);

        // Get Difference in Velocities of points A and B
        pDIFFERENCE(jello->v[i][j][k], jello->v[i+1][j+1][k-1], vDiff);

        // Calculate Forces
        point hookForce = calcHookForce(i, j, k, jello, l, mainRestLength); // Hook's Law
        point dampForce = calcDampForce(i, j, k, jello, l, vDiff);          // Damping

        // Accumulate Forces
        pSUM(shearForce, hookForce, shearForce);
        pSUM(shearForce, dampForce, shearForce);
    }

    // Main Diagonal Case 3
    if((i != 7) && (j != 0) && (k != 7))
    {
        // Get Vector L (Vector pointing from B to A)
        pDIFFERENCE(jello->p[i][j][k], jello->p[i+1][j-1][k+1], l);

        // Get Difference in Velocities of points A and B
        pDIFFERENCE(jello->v[i][j][k], jello->v[i+1][j-1][k+1], vDiff);

        // Calculate Forces
        point hookForce = calcHookForce(i, j, k, jello, l, mainRestLength); // Hook's Law
        point dampForce = calcDampForce(i, j, k, jello, l, vDiff);          // Damping

        // Accumulate Forces
        pSUM(shearForce, hookForce, shearForce);
        pSUM(shearForce, dampForce, shearForce);
    }

    // Main Diagonal Case 4
    if((i != 0) && (j != 7) && (k != 7))
    {
        // Get Vector L (Vector pointing from B to A)
        pDIFFERENCE(jello->p[i][j][k], jello->p[i-1][j+1][k+1], l);

        // Get Difference in Velocities of points A and B
        pDIFFERENCE(jello->v[i][j][k], jello->v[i-1][j+1][k+1], vDiff);

        // Calculate Forces
        point hookForce = calcHookForce(i, j, k, jello, l, mainRestLength); // Hook's Law
        point dampForce = calcDampForce(i, j, k, jello, l, vDiff);          // Damping

        // Accumulate Forces
        pSUM(shearForce, hookForce, shearForce);
        pSUM(shearForce, dampForce, shearForce);
    }

    // Main Diagonal Case 5
    if((i != 0) && (j != 0) && (k != 7))
    {
        // Get Vector L (Vector pointing from B to A)
        pDIFFERENCE(jello->p[i][j][k], jello->p[i-1][j-1][k+1], l);

        // Get Difference in Velocities of points A and B
        pDIFFERENCE(jello->v[i][j][k], jello->v[i-1][j-1][k+1], vDiff);

        // Calculate Forces
        point hookForce = calcHookForce(i, j, k, jello, l, mainRestLength); // Hook's Law
        point dampForce = calcDampForce(i, j, k, jello, l, vDiff);          // Damping

        // Accumulate Forces
        pSUM(shearForce, hookForce, shearForce);
        pSUM(shearForce, dampForce, shearForce);
    }

    // Main Diagonal Case 6
    if((i != 0) && (j != 7) && (k != 0))
    {
        // Get Vector L (Vector pointing from B to A)
        pDIFFERENCE(jello->p[i][j][k], jello->p[i-1][j+1][k-1], l);

        // Get Difference in Velocities of points A and B
        pDIFFERENCE(jello->v[i][j][k], jello->v[i-1][j+1][k-1], vDiff);

        // Calculate Forces
        point hookForce = calcHookForce(i, j, k, jello, l, mainRestLength); // Hook's Law
        point dampForce = calcDampForce(i, j, k, jello, l, vDiff);          // Damping

        // Accumulate Forces
        pSUM(shearForce, hookForce, shearForce);
        pSUM(shearForce, dampForce, shearForce);
    }

    // Main Diagonal Case 7
    if((i != 0) && (j != 0) && (k != 0))
    {
        // Get Vector L (Vector pointing from B to A)
        pDIFFERENCE(jello->p[i][j][k], jello->p[i-1][j-1][k-1], l);

        // Get Difference in Velocities of points A and B
        pDIFFERENCE(jello->v[i][j][k], jello->v[i-1][j-1][k-1], vDiff);

        // Calculate Forces
        point hookForce = calcHookForce(i, j, k, jello, l, mainRestLength); // Hook's Law
        point dampForce = calcDampForce(i, j, k, jello, l, vDiff);          // Damping

        // Accumulate Forces
        pSUM(shearForce, hookForce, shearForce);
        pSUM(shearForce, dampForce, shearForce);
    }

    // Main Diagonal Case 8
    if((i != 7) && (j != 0) && (k != 0))
    {
        // Get Vector L (Vector pointing from B to A)
        pDIFFERENCE(jello->p[i][j][k], jello->p[i+1][j-1][k-1], l);

        // Get Difference in Velocities of points A and B
        pDIFFERENCE(jello->v[i][j][k], jello->v[i+1][j-1][k-1], vDiff);

        // Calculate Forces
        point hookForce = calcHookForce(i, j, k, jello, l, mainRestLength); // Hook's Law
        point dampForce = calcDampForce(i, j, k, jello, l, vDiff);          // Damping

        // Accumulate Forces
        pSUM(shearForce, hookForce, shearForce);
        pSUM(shearForce, dampForce, shearForce);
    }

    return shearForce;
}

/**
 * processBendSprings - Computes all the forces acting
 *                      on the Bend Springs
 */
struct point processBendSprings(int i, int j, int k, struct world *jello)
{
    // Initialize Struct Force
    point bendForce;

    // Initilaize Attributes
    point l;     // L Vector
    point vDiff; // Velocity Difference

    // Initialize Rest Length
    double restLength = (2.0/7.0);

    // Check Neighbor 1 Case
    if(i < 6)
    {
        // Get Vector L (Vector pointing from B to A)
        pDIFFERENCE(jello->p[i][j][k], jello->p[i+2][j][k], l);

        // Get Difference in Velocities of points A and B
        pDIFFERENCE(jello->v[i][j][k], jello->v[i+2][j][k], vDiff);

        // Calculate Forces
        point hookForce = calcHookForce(i, j, k, jello, l, restLength); // Hook's Law
        point dampForce = calcDampForce(i, j, k, jello, l, vDiff);      // Damping

        // Accumulate Forces
        pSUM(bendForce, hookForce, bendForce);
        pSUM(bendForce, dampForce, bendForce);
    }

    // Check Neighbor 2 Case
    if(i > 1)
    {
        // Get Vector L (Vector pointing from B to A)
        pDIFFERENCE(jello->p[i][j][k], jello->p[i-2][j][k], l);

        // Get Difference in Velocities of points A and B
        pDIFFERENCE(jello->v[i][j][k], jello->v[i-2][j][k], vDiff);

        // Calculate Forces
        point hookForce = calcHookForce(i, j, k, jello, l, restLength); // Hook's Law
        point dampForce = calcDampForce(i, j, k, jello, l, vDiff);      // Damping

        // Accumulate Forces
        pSUM(bendForce, hookForce, bendForce);
        pSUM(bendForce, dampForce, bendForce);
    }

    // Check Neighbor 3 Case
    if(j < 6)
    {
        // Get Vector L (Vector pointing from B to A)
        pDIFFERENCE(jello->p[i][j][k], jello->p[i][j+2][k], l);

        // Get Difference in Velocities of points A and B
        pDIFFERENCE(jello->v[i][j][k], jello->v[i][j+2][k], vDiff);

        // Calculate Forces
        point hookForce = calcHookForce(i, j, k, jello, l, restLength); // Hook's Law
        point dampForce = calcDampForce(i, j, k, jello, l, vDiff);      // Damping

        // Accumulate Forces
        pSUM(bendForce, hookForce, bendForce);
        pSUM(bendForce, dampForce, bendForce);
    }

    // Check Neighbor 4 Case
    if(j > 1)
    {
        // Get Vector L (Vector pointing from B to A)
        pDIFFERENCE(jello->p[i][j][k], jello->p[i][j-2][k], l);

        // Get Difference in Velocities of points A and B
        pDIFFERENCE(jello->v[i][j][k], jello->v[i][j-2][k], vDiff);

        // Calculate Forces
        point hookForce = calcHookForce(i, j, k, jello, l, restLength); // Hook's Law
        point dampForce = calcDampForce(i, j, k, jello, l, vDiff);      // Damping

        // Accumulate Forces
        pSUM(bendForce, hookForce, bendForce);
        pSUM(bendForce, dampForce, bendForce);
    }

    // Check Neighbor 5 Case
    if(k < 6)
    {
        // Get Vector L (Vector pointing from B to A)
        pDIFFERENCE(jello->p[i][j][k], jello->p[i][j][k+2], l);

        // Get Difference in Velocities of points A and B
        pDIFFERENCE(jello->v[i][j][k], jello->v[i][j][k+2], vDiff);

        // Calculate Forces
        point hookForce = calcHookForce(i, j, k, jello, l, restLength); // Hook's Law
        point dampForce = calcDampForce(i, j, k, jello, l, vDiff);      // Damping

        // Accumulate Forces
        pSUM(bendForce, hookForce, bendForce);
        pSUM(bendForce, dampForce, bendForce);
    }

    // Check Neighbor 6 Case
    if(k > 1)
    {
        // Get Vector L (Vector pointing from B to A)
        pDIFFERENCE(jello->p[i][j][k], jello->p[i][j][k-2], l);

        // Get Difference in Velocities of points A and B
        pDIFFERENCE(jello->v[i][j][k], jello->v[i][j][k-2], vDiff);

        // Calculate Forces
        point hookForce = calcHookForce(i, j, k, jello, l, restLength); // Hook's Law
        point dampForce = calcDampForce(i, j, k, jello, l, vDiff);      // Damping

        // Accumulate Forces
        pSUM(bendForce, hookForce, bendForce);
        pSUM(bendForce, dampForce, bendForce);
    }

    return bendForce;
}

/**
 * computeAcceleration - Computes acceleration to every control
 *                       point of the jello cube, which is in
 *                       state given by 'jello'
 *
 * @return - Returns result in array 'a'.
 */
void computeAcceleration(struct world *jello, struct point a[8][8][8])
{
    // Get the Mass of the Mass Point
    double m = jello->mass;

    // Indicator to only allow for one Collision
    bool collisionCheck = true;

    // Iterate over points of Cube
    for (int i=0; i<=7; i++)
    {
        for (int j=0; j<=7; j++)
        {
            for (int k=0; k<=7; k++)
            {
                // Initialize Total Force
                point totalForce;
                totalForce.x = 0.0;
                totalForce.y = 0.0;
                totalForce.z = 0.0;

                // Check if there is a Collision
                if(checkCollision(i,j,k, jello) && collisionCheck)
                {
                    // Only Update One Mass Point
                    collisionCheck = false;

                    // TODO: Remove Debug Statement
                    std::cout << "Collision Occurred: (" << i << "," << j << "," << k << ")" << std::endl;

                    // Process Collision Force
                    point collisionForce = processCollision(i,j,k,jello);

                    // Add Collision Force to total force
                    pSUM(totalForce, collisionForce, totalForce);
                }

                // Process Spring Forces
                point structForce = processStructSprings(i,j,k,jello);
                point shearForce = processShearSprings(i,j,k,jello);
                point bendForce = processBendSprings(i,j,k,jello);

                // Accumulate Forces
                pSUM(totalForce, structForce, totalForce);
                pSUM(totalForce, shearForce, totalForce);
                pSUM(totalForce, bendForce, totalForce);

                // Get the Acceleration
                point acceleration;
                pMULTIPLY(totalForce, (1/m), a[i][j][k]);
            }
        }
    }
}

/**
 * Euler - Performs one step of Euler Integration
 *         as a result, updates the jello structure
 */
void Euler(struct world *jello)
{
    // Create iterators
    int i,j,k;

    // Create Acceleration
    point a[8][8][8];

    // Calculate the Acceleration for the Mass Point System
    computeAcceleration(jello, a);

    // Iterate over X Dimension of Mass Points
    for (i=0; i<=7; i++)
    {
        // Iterate over Y Dimension of Mass Points
        for (j=0; j<=7; j++)
        {
            // Iterate over Z Dimension of Mass Points
            for (k=0; k<=7; k++)
            {
                jello->p[i][j][k].x += jello->dt * jello->v[i][j][k].x;
                jello->p[i][j][k].y += jello->dt * jello->v[i][j][k].y;
                jello->p[i][j][k].z += jello->dt * jello->v[i][j][k].z;
                jello->v[i][j][k].x += jello->dt * a[i][j][k].x;
                jello->v[i][j][k].y += jello->dt * a[i][j][k].y;
                jello->v[i][j][k].z += jello->dt * a[i][j][k].z;

            }
        }
    }
}

/**
 * RK4 - Performs one step of RK4 Integration
 *       as a result, updates the jello structure
 */
void RK4(struct world * jello)
{
    point F1p[8][8][8], F1v[8][8][8],
          F2p[8][8][8], F2v[8][8][8],
          F3p[8][8][8], F3v[8][8][8],
          F4p[8][8][8], F4v[8][8][8];

    // Create Acceleration
    point a[8][8][8];

    struct world buffer;

    // Create iterators
    int i,j,k;

    // Make a copy of jello
    buffer = *jello;

    // Calculate the Acceleration for the Mass Point System
    computeAcceleration(jello, a);

    // Iterate over X Dimension of Mass Points
    for (i=0; i<=7; i++)
    {
        // Iterate over Y Dimension of Mass Points
        for (j=0; j<=7; j++)
        {
            // Iterate over Z Dimension of Mass Points
            for (k=0; k<=7; k++)
            {
                pMULTIPLY(jello->v[i][j][k],jello->dt,F1p[i][j][k]);
                pMULTIPLY(a[i][j][k],jello->dt,F1v[i][j][k]);
                pMULTIPLY(F1p[i][j][k],0.5,buffer.p[i][j][k]);
                pMULTIPLY(F1v[i][j][k],0.5,buffer.v[i][j][k]);
                pSUM(jello->p[i][j][k],buffer.p[i][j][k],buffer.p[i][j][k]);
                pSUM(jello->v[i][j][k],buffer.v[i][j][k],buffer.v[i][j][k]);
            }
        }
    }

    computeAcceleration(&buffer, a);

    for (i=0; i<=7; i++)
        for (j=0; j<=7; j++)
            for (k=0; k<=7; k++)
            {
                // F2p = dt * buffer.v;
                pMULTIPLY(buffer.v[i][j][k],jello->dt,F2p[i][j][k]);
                // F2v = dt * a(buffer.p,buffer.v);
                pMULTIPLY(a[i][j][k],jello->dt,F2v[i][j][k]);
                pMULTIPLY(F2p[i][j][k],0.5,buffer.p[i][j][k]);
                pMULTIPLY(F2v[i][j][k],0.5,buffer.v[i][j][k]);
                pSUM(jello->p[i][j][k],buffer.p[i][j][k],buffer.p[i][j][k]);
                pSUM(jello->v[i][j][k],buffer.v[i][j][k],buffer.v[i][j][k]);
            }

    computeAcceleration(&buffer, a);

    for (i=0; i<=7; i++)
        for (j=0; j<=7; j++)
            for (k=0; k<=7; k++)
            {
                // F3p = dt * buffer.v;
                pMULTIPLY(buffer.v[i][j][k],jello->dt,F3p[i][j][k]);
                // F3v = dt * a(buffer.p,buffer.v);
                pMULTIPLY(a[i][j][k],jello->dt,F3v[i][j][k]);
                pMULTIPLY(F3p[i][j][k],1.0,buffer.p[i][j][k]);
                pMULTIPLY(F3v[i][j][k],1.0,buffer.v[i][j][k]);
                pSUM(jello->p[i][j][k],buffer.p[i][j][k],buffer.p[i][j][k]);
                pSUM(jello->v[i][j][k],buffer.v[i][j][k],buffer.v[i][j][k]);
            }

    computeAcceleration(&buffer, a);


    for (i=0; i<=7; i++)
        for (j=0; j<=7; j++)
            for (k=0; k<=7; k++)
            {
                // F3p = dt * buffer.v;
                pMULTIPLY(buffer.v[i][j][k],jello->dt,F4p[i][j][k]);
                // F3v = dt * a(buffer.p,buffer.v);
                pMULTIPLY(a[i][j][k],jello->dt,F4v[i][j][k]);

                pMULTIPLY(F2p[i][j][k],2,buffer.p[i][j][k]);
                pMULTIPLY(F3p[i][j][k],2,buffer.v[i][j][k]);
                pSUM(buffer.p[i][j][k],buffer.v[i][j][k],buffer.p[i][j][k]);
                pSUM(buffer.p[i][j][k],F1p[i][j][k],buffer.p[i][j][k]);
                pSUM(buffer.p[i][j][k],F4p[i][j][k],buffer.p[i][j][k]);
                pMULTIPLY(buffer.p[i][j][k],1.0 / 6,buffer.p[i][j][k]);
                pSUM(buffer.p[i][j][k],jello->p[i][j][k],jello->p[i][j][k]);

                pMULTIPLY(F2v[i][j][k],2,buffer.p[i][j][k]);
                pMULTIPLY(F3v[i][j][k],2,buffer.v[i][j][k]);
                pSUM(buffer.p[i][j][k],buffer.v[i][j][k],buffer.p[i][j][k]);
                pSUM(buffer.p[i][j][k],F1v[i][j][k],buffer.p[i][j][k]);
                pSUM(buffer.p[i][j][k],F4v[i][j][k],buffer.p[i][j][k]);
                pMULTIPLY(buffer.p[i][j][k],1.0 / 6,buffer.p[i][j][k]);
                pSUM(buffer.p[i][j][k],jello->v[i][j][k],jello->v[i][j][k]);
            }

    return;
}
