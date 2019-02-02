// Headers
#include "jello.h"
#include "physics.h"
#include <string>
#include <iostream>
#include <vector>

/**
 * calcDampForce - Calculates the Damping Force
 *                 on a Mass Point
 */
struct point calcDampForce(struct point pos, double k, struct point l, struct point vDiff)
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
    double kLength = -k * normProjection;

    // Ensure Numerics aren't zero to prevent divide by zero case
    if((pos.x != 0.0) || (pos.y != 0.0) || (pos.z != 0.0))
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
struct point calcHookForce(point pos, double kHook, point l, double rLength)
{
    // Initialize Hook Force
    point hookForce;

    // Get the Magnitude of the Length
    double mag;
    pMAG(l, mag);

    // Calculate the Length (|L| - R)
    double dX = mag - rLength;

    // Multiply by Hook's Constant -kHook(|L| - R)
    double kLength = -kHook * dX;

    // Ensure Numerics aren't zero to prevent divide by zero case
    if((pos.x != 0.0) || (pos.y != 0.0) || (pos.z != 0.0))
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
 * calcExternalForce - Calculates the Force exerted
 *                     by an External Force Field
 *                     acting on the Mass Point
 */
struct point calcExternalForce(int i, int j, int k, struct world *jello)
{
    //Initialize External Force
    point extForce;
    extForce.x = 0.0;
    extForce.y = 0.0;
    extForce.z = 0.0;

    // Get Resolution
    int res = jello->resolution;

    // Check if there is a Force Field
    if(res != 0)
    {
        // Get Position of Mass Point
        point pos = jello->p[i][j][k];

        if(pos.x >= 2)
        {
            pos.x = 2;
        }

        if(pos.x <= -2)
        {
            pos.x = -2;
        }

        if(pos.y >= 2)
        {
            pos.y = 2;
        }

        if(pos.y <= -2)
        {
            pos.y = -2;
        }

        if(pos.z >= 2)
        {
            pos.z = 2;
        }

        if(pos.z <= -2)
        {
            pos.z = -2;
        }

        // Map Input Range of Bounding Box [-2,2] to Output Range of Force Field [0, resolution-1]
        double x = (pos.x + 2) * ((res-1)/4);
        double y = (pos.y + 2) * ((res-1)/4);
        double z = (pos.z + 2) * ((res-1)/4);

        // Cast to Int so it can be indexed into Force Field
        int i = round(x);
        int j = round(y);
        int k = round(z);

        // Get the Eight Vertices surrounding the Force Field Cube
        point c000 = jello->forceField[(i*res*res) + (j*res) + k];
        point c001 = jello->forceField[(i*res*res) + (j*res) + (k+1)];
        point c010 = jello->forceField[(i*res*res) + ((j+1)*res) + k];
        point c100 = jello->forceField[((i+1)*res*res) + (j*res) + k];
        point c101 = jello->forceField[((i+1)*res*res) + (j*res) + (k+1)];
        point c110 = jello->forceField[((i+1)*res*res) + ((j+1)*res) + k];
        point c011 = jello->forceField[(i*res*res) + ((j+1)*res) + (k+1)];
        point c111 = jello->forceField[((i+1)*res*res) + ((j+1)*res) + (k+1)];

        double tx = x - i;
        double ty = y - j;
        double tz = z - k;

        // Calculate External Force on X Position
        extForce.x = ((1-tx) * (1-ty) * (1-tz) * c000.x) +
                     ((1-tx) * (1-ty) * tz * c001.x) +
                     ((1-tx) * ty * (1-tz) * c010.x) +
                     (tx * (1-ty) * (1-tz) * c100.x) +
                     (tx * (1-ty) * tz * c101.x) +
                     ((1-tx) * ty * tz * c011.x) +
                     (tx * ty * (1-tz) * c110.x) +
                     (tx * ty * tz * c111.x);

        // Calculate External Force on Y Position
        extForce.y = ((1-tx) * (1-ty) * (1-tz) * c000.y) +
                     ((1-tx) * (1-ty) * tz * c001.y) +
                     ((1-tx) * ty * (1-tz) * c010.y) +
                     (tx * (1-ty) * (1-tz) * c100.y) +
                     (tx * (1-ty) * tz * c101.y) +
                     ((1-tx) * ty * tz * c011.y) +
                     (tx * ty * (1-tz) * c110.y) +
                     (tx * ty * tz * c111.y);

        // Calculate External Force on Z Position
        extForce.z = ((1-tx) * (1-ty) * (1-tz) * c000.z) +
                     ((1-tx) * (1-ty) * tz * c001.z) +
                     ((1-tx) * ty * (1-tz) * c010.x) +
                     (tx * (1-ty) * (1-tz) * c100.z) +
                     (tx * (1-ty) * tz * c101.z) +
                     ((1-tx) * ty * tz * c011.z) +
                     (tx * ty * (1-tz) * c110.z) +
                     (tx * ty * tz * c111.z);
    }

    return extForce;
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
 * processCollision - Process Penalty Force based on Collision
 *                    with the Bouding Box
 */
struct point processCollision(int i, int j, int k, struct world *jello)
{
    // Initialize Collision Force
    point collisionForce;
    collisionForce.x = 0.0;
    collisionForce.y = 0.0;
    collisionForce.z = 0.0;

    // Initialize Rest Length (0 for a Collision Spring)
    double restLength = 0.0;

    // Check Out of Bounds Case 1 (X Min)
    if(jello->p[i][j][k].x <= -2.0)
    {
        // Make Bounding Point
        struct point xMinBound;
        xMinBound.x = -2.0;
        xMinBound.y =  jello->p[i][j][k].y;
        xMinBound.z =  jello->p[i][j][k].z;

        // Get Vector L (Vector pointing from B to A)
        point l;
        pDIFFERENCE(jello->p[i][j][k], xMinBound, l);

        // Calculate Forces
        point hookForce = calcHookForce(jello->p[i][j][k], jello->kCollision, l, restLength);        // Hook's Law
        point dampForce = calcDampForce(jello->p[i][j][k], jello->dCollision, l, jello->v[i][j][k]); // Damping

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
        xMaxBound.y = jello->p[i][j][k].y;
        xMaxBound.z = jello->p[i][j][k].z;

        // Get Vector L (Vector pointing from B to A)
        point l;
        pDIFFERENCE(jello->p[i][j][k], xMaxBound, l);

        // Calculate Forces
        point hookForce = calcHookForce(jello->p[i][j][k], jello->kCollision, l, restLength);        // Hook's Law
        point dampForce = calcDampForce(jello->p[i][j][k], jello->dCollision, l, jello->v[i][j][k]); // Damping

        // Accumulate Forces
        pSUM(collisionForce, hookForce, collisionForce);
        pSUM(collisionForce, dampForce, collisionForce);
    }

    // Check Out of Bounds Case 3 (Y Min)
    if(jello->p[i][j][k].y <= -2.0)
    {
        // Make Bounding Point
        struct point yMinBound;
        yMinBound.x =  jello->p[i][j][k].x;
        yMinBound.y = -2.0;
        yMinBound.z =  jello->p[i][j][k].z;

        // Get Vector L (Vector pointing from B to A)
        point l;
        pDIFFERENCE(jello->p[i][j][k], yMinBound, l);

        // Calculate Forces
        point hookForce = calcHookForce(jello->p[i][j][k], jello->kCollision, l, restLength);        // Hook's Law
        point dampForce = calcDampForce(jello->p[i][j][k], jello->dCollision, l, jello->v[i][j][k]); // Damping

        // Accumulate Forces
        pSUM(collisionForce, hookForce, collisionForce);
        pSUM(collisionForce, dampForce, collisionForce);
    }

    // Check Out of Bounds Case 4 (Y Max)
    if(jello->p[i][j][k].y >= 2.0)
    {
        // Make Bounding Point
        struct point yMaxBound;
        yMaxBound.x = jello->p[i][j][k].x;
        yMaxBound.y = 2.0;
        yMaxBound.z = jello->p[i][j][k].z;

        // Get Vector L (Vector pointing from B to A)
        point l;
        pDIFFERENCE(jello->p[i][j][k], yMaxBound, l);

        // Calculate Forces
        point hookForce = calcHookForce(jello->p[i][j][k], jello->kCollision, l, restLength);        // Hook's Law
        point dampForce = calcDampForce(jello->p[i][j][k], jello->dCollision, l, jello->v[i][j][k]); // Damping

        // Accumulate Forces
        pSUM(collisionForce, hookForce, collisionForce);
        pSUM(collisionForce, dampForce, collisionForce);
    }

    // Check Out of Bounds Case 5 (Z Min)
    if(jello->p[i][j][k].z <= -2.0)
    {
        // Make Bounding Point
        struct point zMinBound;
        zMinBound.x = jello->p[i][j][k].x;
        zMinBound.y = jello->p[i][j][k].y;
        zMinBound.z = -2.0;

        // Get Vector L (Vector pointing from B to A)
        point l;
        pDIFFERENCE(jello->p[i][j][k], zMinBound, l);

        // Calculate Forces
        point hookForce = calcHookForce(jello->p[i][j][k], jello->kCollision, l, restLength);        // Hook's Law
        point dampForce = calcDampForce(jello->p[i][j][k], jello->dCollision, l, jello->v[i][j][k]); // Damping

        // Accumulate Forces
        pSUM(collisionForce, hookForce, collisionForce);
        pSUM(collisionForce, dampForce, collisionForce);
    }

    // Check Out of Bounds Case 6 (Z Max)
    if(jello->p[i][j][k].z >= 2.0)
    {
        // Make Bounding Point
        struct point zMaxBound;
        zMaxBound.x = jello->p[i][j][k].x;
        zMaxBound.y = jello->p[i][j][k].y;
        zMaxBound.z = 2.0;

        // Get Vector L (Vector pointing from B to A)
        point l;
        pDIFFERENCE(jello->p[i][j][k], zMaxBound, l);

        // Calculate Forces
        point hookForce = calcHookForce(jello->p[i][j][k], jello->kCollision, l, restLength);        // Hook's Law
        point dampForce = calcDampForce(jello->p[i][j][k], jello->dCollision, l, jello->v[i][j][k]); // Damping

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
    structForce.x = 0.0;
    structForce.y = 0.0;
    structForce.z = 0.0;

    // Initialize Rest Length
    double restLength = 1.0/7.0;

    // Initialize Neighbor Array
    std::vector<particle> neighbors;

    // Check Neighbor 1 Case
    if(i != 7)
    {
        // Create Mass Point
        particle massPoint;
        pPARTICLE(jello->p[i+1][j][k], jello->v[i+1][j][k], massPoint);

        // Add to Neighbors Array
        neighbors.push_back(massPoint);
    }

    // Check Neighbor 2 Case
    if(i != 0)
    {
        // Create Mass Point
        particle massPoint;
        pPARTICLE(jello->p[i-1][j][k], jello->v[i-1][j][k], massPoint);

        // Add to Neighbors Array
        neighbors.push_back(massPoint);
    }

    // Check Neighbor 3 Case
    if(j != 7)
    {
        // Create Mass Point
        particle massPoint;
        pPARTICLE(jello->p[i][j+1][k], jello->v[i][j+1][k], massPoint);

        // Add to Neighbors Array
        neighbors.push_back(massPoint);
    }

    // Check Neighbor 4 Case
    if(j != 0)
    {
        // Create Mass Point
        particle massPoint;
        pPARTICLE(jello->p[i][j-1][k], jello->v[i][j-1][k], massPoint);

        // Add to Neighbors Array
        neighbors.push_back(massPoint);
    }

    // Check Neighbor 5 Case
    if(k != 7)
    {
        // Create Mass Point
        particle massPoint;
        pPARTICLE(jello->p[i][j][k+1], jello->v[i][j][k+1], massPoint);

        // Add to Neighbors Array
        neighbors.push_back(massPoint);
    }

    // Check Neighbor 6 Case
    if(k != 0)
    {
        // Create Mass Point
        particle massPoint;
        pPARTICLE(jello->p[i][j][k-1], jello->v[i][j][k-1], massPoint);

        // Add to Neighbors Array
        neighbors.push_back(massPoint);
    }

    // Create Iterator
    std::vector<particle>::iterator neighbor;

    // Iterate over the Neighbors
    for(neighbor = neighbors.begin(); neighbor != neighbors.end(); ++neighbor)
    {
        // Get Vector L (Vector pointing from B to A)
        point l;
        pDIFFERENCE(jello->p[i][j][k], neighbor->position, l);

        // Get Difference in Velocities of points A and B
        point vDiff;
        pDIFFERENCE(jello->v[i][j][k], neighbor->velocity, vDiff);

        // Calculate Forces
        point hookForce = calcHookForce(jello->p[i][j][k], jello->kElastic, l, restLength); // Hook's Law
        point dampForce = calcDampForce(jello->p[i][j][k], jello->dElastic, l, vDiff);      // Damping

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
    shearForce.x = 0.0;
    shearForce.y = 0.0;
    shearForce.z = 0.0;

    // Initialize Rest Lengths
    double mainRestLength = (1.0/7.0) * (sqrt(3));
    double sideRestLength = (1.0/7.0) * (sqrt(2));

    // Initialize Neighbor Arrays
    std::vector<particle> sideNeighbors;
    std::vector<particle> mainNeighbors;

    // Side Diagonal Case 1
    if((i != 7) && (j != 7))
    {
        // Create Mass Point
        particle massPoint;
        pPARTICLE(jello->p[i+1][j+1][k], jello->v[i+1][j+1][k], massPoint);

        // Add to Side Neighbors Array
        sideNeighbors.push_back(massPoint);
    }

    // Side Diagonal Case 2
    if((i != 0) && (j != 7))
    {
        // Create Mass Point
        particle massPoint;
        pPARTICLE(jello->p[i-1][j+1][k], jello->v[i-1][j+1][k], massPoint);

        // Add to Side Neighbors Array
        sideNeighbors.push_back(massPoint);
    }

    // Side Diagonal Case 3
    if((i != 7) && (j != 0))
    {
        // Create Mass Point
        particle massPoint;
        pPARTICLE(jello->p[i+1][j-1][k], jello->v[i+1][j-1][k], massPoint);

        // Add to Side Neighbors Array
        sideNeighbors.push_back(massPoint);
    }

    // Side Diagonal Case 4
    if((i != 0) && (j != 0))
    {
        // Create Mass Point
        particle massPoint;
        pPARTICLE(jello->p[i-1][j-1][k], jello->v[i-1][j-1][k], massPoint);

        // Add to Side Neighbors Array
        sideNeighbors.push_back(massPoint);
    }

    // Side Diagonal Case 5
    if((j != 7) && (k != 7))
    {
        // Create Mass Point
        particle massPoint;
        pPARTICLE(jello->p[i][j+1][k+1], jello->v[i][j+1][k+1], massPoint);

        // Add to Side Neighbors Array
        sideNeighbors.push_back(massPoint);
    }

    // Side Diagonal Case 6
    if((j != 0) && (k != 7))
    {
        // Create Mass Point
        particle massPoint;
        pPARTICLE(jello->p[i][j-1][k+1], jello->v[i][j-1][k+1], massPoint);

        // Add to Side Neighbors Array
        sideNeighbors.push_back(massPoint);
    }

    // Side Diagonal Case 7
    if((j != 7) && (k != 0))
    {
        // Create Mass Point
        particle massPoint;
        pPARTICLE(jello->p[i][j+1][k-1], jello->v[i][j+1][k-1], massPoint);

        // Add to Side Neighbors Array
        sideNeighbors.push_back(massPoint);
    }

    // Side Diagonal Case 8
    if((j != 0) && (k != 0))
    {
        // Create Mass Point
        particle massPoint;
        pPARTICLE(jello->p[i][j-1][k-1], jello->v[i][j-1][k-1], massPoint);

        // Add to Side Neighbors Array
        sideNeighbors.push_back(massPoint);
    }

    // Side Diagonal Case 9
    if((i != 7) && (k != 7))
    {
        // Create Mass Point
        particle massPoint;
        pPARTICLE(jello->p[i+1][j][k+1], jello->v[i+1][j][k+1], massPoint);

        // Add to Side Neighbors Array
        sideNeighbors.push_back(massPoint);
    }

    // Side Diagonal Case 10
    if((i != 0) && (k != 7))
    {
        // Create Mass Point
        particle massPoint;
        pPARTICLE(jello->p[i-1][j][k+1], jello->v[i-1][j][k+1], massPoint);

        // Add to Side Neighbors Array
        sideNeighbors.push_back(massPoint);
    }

    // Side Diagonal Case 11
    if((i != 7) && (k != 0))
    {
        // Create Mass Point
        particle massPoint;
        pPARTICLE(jello->p[i+1][j][k-1], jello->v[i+1][j][k-1], massPoint);

        // Add to Side Neighbors Array
        sideNeighbors.push_back(massPoint);
    }

    // Side Diagonal Case 12
    if((i != 0) && (k != 0))
    {
        // Create Mass Point
        particle massPoint;
        pPARTICLE(jello->p[i-1][j][k-1], jello->v[i-1][j][k-1], massPoint);

        // Add to Side Neighbors Array
        sideNeighbors.push_back(massPoint);
    }

    // Main Diagonal Case 1
    if((i != 7) && (j != 7) && (k != 7))
    {
        // Create Mass Point
        particle massPoint;
        pPARTICLE(jello->p[i+1][j+1][k+1], jello->v[i+1][j+1][k+1], massPoint);

        // Add to Main Neighbors Array
        mainNeighbors.push_back(massPoint);
    }

    // Main Diagonal Case 2
    if((i != 7) && (j != 7) && (k != 0))
    {
        // Create Mass Point
        particle massPoint;
        pPARTICLE(jello->p[i+1][j+1][k-1], jello->v[i+1][j+1][k-1], massPoint);

        // Add to Main Neighbors Array
        mainNeighbors.push_back(massPoint);
    }

    // Main Diagonal Case 3
    if((i != 7) && (j != 0) && (k != 7))
    {
        // Create Mass Point
        particle massPoint;
        pPARTICLE(jello->p[i+1][j-1][k+1], jello->v[i+1][j-1][k+1], massPoint);

        // Add to Main Neighbors Array
        mainNeighbors.push_back(massPoint);
    }

    // Main Diagonal Case 4
    if((i != 0) && (j != 7) && (k != 7))
    {
        // Create Mass Point
        particle massPoint;
        pPARTICLE(jello->p[i-1][j+1][k+1], jello->v[i-1][j+1][k+1], massPoint);

        // Add to Main Neighbors Array
        mainNeighbors.push_back(massPoint);
    }

    // Main Diagonal Case 5
    if((i != 0) && (j != 0) && (k != 7))
    {
        // Create Mass Point
        particle massPoint;
        pPARTICLE(jello->p[i-1][j-1][k+1], jello->v[i-1][j-1][k+1], massPoint);

        // Add to Main Neighbors Array
        mainNeighbors.push_back(massPoint);
    }

    // Main Diagonal Case 6
    if((i != 0) && (j != 7) && (k != 0))
    {
        // Create Mass Point
        particle massPoint;
        pPARTICLE(jello->p[i-1][j+1][k-1], jello->v[i-1][j+1][k-1], massPoint);

        // Add to Main Neighbors Array
        mainNeighbors.push_back(massPoint);
    }

    // Main Diagonal Case 7
    if((i != 0) && (j != 0) && (k != 0))
    {
        // Create Mass Point
        particle massPoint;
        pPARTICLE(jello->p[i-1][j-1][k-1], jello->v[i-1][j-1][k-1], massPoint);

        // Add to Main Neighbors Array
        mainNeighbors.push_back(massPoint);
    }

    // Main Diagonal Case 8
    if((i != 7) && (j != 0) && (k != 0))
    {
        // Create Mass Point
        particle massPoint;
        pPARTICLE(jello->p[i+1][j-1][k-1], jello->v[i+1][j-1][k-1], massPoint);

        // Add to Main Neighbors Array
        mainNeighbors.push_back(massPoint);
    }

    // Create Iterators
    std::vector<particle>::iterator sideNeighbor;
    std::vector<particle>::iterator mainNeighbor;

    // Iterate over the Side Diagonal Neighbor Positions
    for(sideNeighbor = sideNeighbors.begin(); sideNeighbor != sideNeighbors.end(); ++sideNeighbor)
    {
        // Initilaize Attributes
        point l;     // L Vector
        point vDiff; // Velocity Difference

        // Get Vector L (Vector pointing from B to A)
        pDIFFERENCE(jello->p[i][j][k], sideNeighbor->position, l);

        // Get Difference in Velocities of points A and B
        pDIFFERENCE(jello->v[i][j][k], sideNeighbor->velocity, vDiff);

        // Calculate Forces
        point hookForce = calcHookForce(jello->p[i][j][k], jello->kElastic, l, sideRestLength); // Hook's Law
        point dampForce = calcDampForce(jello->p[i][j][k], jello->dElastic, l, vDiff);          // Damping

        // Accumulate Forces
        pSUM(shearForce, hookForce, shearForce);
        pSUM(shearForce, dampForce, shearForce);
    }

    // Iterate over the Main Diagonal Neighbor Positions
    for(mainNeighbor = mainNeighbors.begin(); mainNeighbor != mainNeighbors.end(); ++mainNeighbor)
    {
        // Get Vector L (Vector pointing from B to A)
        point l;
        pDIFFERENCE(jello->p[i][j][k], mainNeighbor->position, l);

        // Get Difference in Velocities of points A and B
        point vDiff;
        pDIFFERENCE(jello->v[i][j][k], mainNeighbor->velocity, vDiff);

        // Calculate Forces
        point hookForce = calcHookForce(jello->p[i][j][k], jello->kElastic, l, mainRestLength); // Hook's Law
        point dampForce = calcDampForce(jello->p[i][j][k], jello->dElastic, l, vDiff);          // Damping

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
    bendForce.x = 0.0;
    bendForce.y = 0.0;
    bendForce.z = 0.0;

    // Initialize Rest Length
    double restLength = (2.0/7.0);

    // Initialize Neighbor Array
    std::vector<particle> neighbors;

    // Check Neighbor 1 Case
    if(i < 6)
    {
        // Create Mass Point
        particle massPoint;
        pPARTICLE(jello->p[i+2][j][k], jello->v[i+2][j][k], massPoint);

        // Add to Neighbors Array
        neighbors.push_back(massPoint);
    }

    // Check Neighbor 2 Case
    if(i > 1)
    {
        // Create Mass Point
        particle massPoint;
        pPARTICLE(jello->p[i-2][j][k], jello->v[i-2][j][k], massPoint);

        // Add to Neighbors Array
        neighbors.push_back(massPoint);
    }

    // Check Neighbor 3 Case
    if(j < 6)
    {
        // Create Mass Point
        particle massPoint;
        pPARTICLE(jello->p[i][j+2][k], jello->v[i][j+2][k], massPoint);

        // Add to Neighbors Array
        neighbors.push_back(massPoint);
    }

    // Check Neighbor 4 Case
    if(j > 1)
    {
        // Create Mass Point
        particle massPoint;
        pPARTICLE(jello->p[i][j-2][k], jello->v[i][j-2][k], massPoint);

        // Add to Neighbors Array
        neighbors.push_back(massPoint);
    }

    // Check Neighbor 5 Case
    if(k < 6)
    {
        // Create Mass Point
        particle massPoint;
        pPARTICLE(jello->p[i][j][k+2], jello->v[i][j][k+2], massPoint);

        // Add to Neighbors Array
        neighbors.push_back(massPoint);
    }

    // Check Neighbor 6 Case
    if(k > 1)
    {
        // Create Mass Point
        particle massPoint;
        pPARTICLE(jello->p[i][j][k-2], jello->v[i][j][k-2], massPoint);

        // Add to Neighbors Array
        neighbors.push_back(massPoint);
    }

    // Create Iterator
    std::vector<particle>::iterator neighbor;

    // Iterate over the Neighbors
    for(neighbor = neighbors.begin(); neighbor != neighbors.end(); ++neighbor)
    {
        // Get Vector L (Vector pointing from B to A)
        point l;
        pDIFFERENCE(jello->p[i][j][k], neighbor->position, l);

        // Get Difference in Velocities of points A and B
        point vDiff;
        pDIFFERENCE(jello->v[i][j][k], neighbor->velocity, vDiff);

        // Calculate Forces
        point hookForce = calcHookForce(jello->p[i][j][k], jello->kElastic, l, restLength); // Hook's Law
        point dampForce = calcDampForce(jello->p[i][j][k], jello->dElastic, l, vDiff);      // Damping

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

    // Iterate over X Dimension of Mass Points
    for (int i=0; i<=7; i++)
    {
        // Iterate over Y Dimension of Mass Points
        for (int j=0; j<=7; j++)
        {
            // Iterate over Z Dimension of Mass Points
            for (int k=0; k<=7; k++)
            {
                // Initialize Total Force
                point totalForce;
                totalForce.x = 0.0;
                totalForce.y = 0.0;
                totalForce.z = 0.0;

                // Check if there is a Collision
                if(checkCollision(i,j,k, jello))
                {
                    // Process Collision Force
                    point collisionForce = processCollision(i,j,k,jello);

                    // Add Collision Force to total force
                    pSUM(totalForce, collisionForce, totalForce);
                }

                // Process Spring Forces
                point structForce = processStructSprings(i,j,k,jello);
                point shearForce = processShearSprings(i,j,k,jello);
                point bendForce = processBendSprings(i,j,k,jello);

                // Process External Forces (Force Field)
                //point extForce = calcExternalForce(i,j,k,jello);

                // Accumulate Forces
                pSUM(totalForce, structForce, totalForce);
                pSUM(totalForce, shearForce, totalForce);
                pSUM(totalForce, bendForce, totalForce);
                //pSUM(totalForce, extForce, totalForce);

                // Get the Acceleration
                point acceleration;
                pMULTIPLY(totalForce, (1/m), acceleration);
                a[i][j][k] = acceleration;
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

    // Initialize Buffer Object
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

    // Store Latest Acceleration in Buffer
    computeAcceleration(&buffer, a);

    // Iterate over X Dimension of Mass Points
    for (i=0; i<=7; i++)
    {
        // Iterate over Y Dimension of Mass Points
        for (j=0; j<=7; j++)
        {
            // Iterate over Z Dimension of Mass Points
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
        }
    }

    // Store Latest Acceleration in Buffer
    computeAcceleration(&buffer, a);

    // Iterate over X Dimension of Mass Points
    for (i=0; i<=7; i++)
    {
        // Iterate over Y Dimension of Mass Points
        for (j=0; j<=7; j++)
        {
            // Iterate over Z Dimension of Mass Points
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
        }
    }

    // Store Latest Acceleration in Buffer
    computeAcceleration(&buffer, a);

    // Iterate over X Dimension of Mass Points
    for (i=0; i<=7; i++)
    {
        // Iterate over Y Dimension of Mass Points
        for (j=0; j<=7; j++)
        {
            // Iterate over Z Dimension of Mass Points
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
        }
    }

    return;
}
