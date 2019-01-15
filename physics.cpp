// Headers
#include "jello.h"
#include "physics.h"

void calcDampingForce()
{

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
    // TODO: Implement
    double mass = jello->mass;

    // Get the Damping Coefficient
    double kD = jello->dElastic;

    // Iterate over points of Cube
    for (int i=0; i<=7; i++)
    {
        for (int j=0; j<=7; j++)
        {
            for (int k=0; k<=7; k++)
            {
                // Calculate Structural Spring Forces
                double structForceX;
                double structForceY;
                double structForceZ;

                // Get Current Mass Point
                double x = jello->p[i][j][k].x;
                double y = jello->p[i][j][k].y;
                double z = jello->p[i][j][k].z;

                struct point lVect;
                struct point vDiff;

                // Check Neighbor 1 Case
                if(i != 7)
                {
                    // Get Vector from Neighbor to Mass Point
                    pDIFFERENCE(jello->p[i][j][k], jello->p[i+1][j][k], lVect);

                    // Get Difference in Velocities of points A and B
                    pDIFFERENCE(jello->v[i][j][k], jello->p[i+1][j][k], vDiff);

                    // Calculate the Damping Force
                    double dampX = kD * pMULTIPLY(lVect, vDiff.x, tempX) * pNORMALIZE(lVect);
                    double dampY = kD * pMULTIPLY(lVect, vDiff.y, tempX) * pNORMALIZE(lVect);
                    double dampZ = kD * pMULTIPLY(lVect, vDiff.z, tempX) * pNORMALIZE(lVect);
                }

                // Check edge case
                if(i != 0)
                {
                    // Get Neighbor 2
                    double x2 = jello->p[i-1][j][k].x;
                    double y2 = jello->p[i-1][j][k].y;
                    double z2 = jello->p[i-1][j][k].z;
                }

                // Check edge case
                if(j != 0)
                {
                    // Get Neighbor 3
                    double x3 = jello->p[i][j-1][k].x;
                    double y3 = jello->p[i][j-1][k].y;
                    double z3 = jello->p[i][j-1][k].z;
                }

                // Check edge case
                if(j != 7)
                {
                    // Get Neighbor 4
                    double x4 = jello->p[i][j+1][k].x;
                    double y4 = jello->p[i][j+1][k].y;
                    double z4 = jello->p[i][j+1][k].z;
                }

                // Check edge case
                if(k != 0)
                {
                    // Get Neighbor 5
                    double x5 = jello->p[i][j][k-1].x;
                    double y5 = jello->p[i][j][k-1].y;
                    double z5 = jello->p[i][j][k-1].z;
                }

                // Check edge case
                if(k != 7)
                {
                    // Get Neighbor 6
                    double x6 = jello->p[i][j][k+1].x;
                    double y6 = jello->p[i][j][k+1].y;
                    double z6 = jello->p[i][j][k+1].z;
                }

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

    point a[8][8][8];

    // Calculate the Acceleration for the point
    computeAcceleration(jello, a);

    for (i=0; i<=7; i++)
    {
        for (j=0; j<=7; j++)
        {
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

/* performs one step of RK4 Integration */
/* as a result, updates the jello structure */
void RK4(struct world * jello)
{
    point F1p[8][8][8], F1v[8][8][8],
          F2p[8][8][8], F2v[8][8][8],
          F3p[8][8][8], F3v[8][8][8],
          F4p[8][8][8], F4v[8][8][8];

    point a[8][8][8];


    struct world buffer;

    // Create iterators
    int i,j,k;

    // Make a copy of jello
    buffer = *jello;

    // Compute Acceleration of Jello Cube
    computeAcceleration(jello, a);

    for (i=0; i<=7; i++)
    {
        for (j=0; j<=7; j++)
        {
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
                pMULTIPLY(F3p[i][j][k],0.5,buffer.p[i][j][k]);
                pMULTIPLY(F3v[i][j][k],0.5,buffer.v[i][j][k]);
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
