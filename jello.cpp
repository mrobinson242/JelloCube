/*                                              */
/*  CSCI 520 Computer Animation and Simulation  */
/*  Assignment 1: Jello Cube                    */
/*  Author: Matthew Robinson                    */
/*  Student Id: 9801107811                      */
/*                                              */

// Headers
#include "jello.h"
#include "showCube.h"
#include "input.h"
#include "physics.h"
#include <iostream>

using namespace std;

// Camera parameters
double Theta = pi/6;
double Phi = pi/6;
double R = 6;

// Mouse control
int g_iMenuId;
int g_vMousePos[2];
int g_iLeftMouseButton,g_iMiddleMouseButton,g_iRightMouseButton;

// Number of images saved to disk so far
int sprite = 0;

// Initialize variables control
// what is displayed on screen
int shear = 0;
int bend = 0;
int structural = 1;
int pause = 0;
int viewingMode = 0;
int saveScreenToFile = 0;

struct world jello;

// Width/Height of Applciation Window
int _windowWidth;
int _windowHeight;

/**
 * init - Initializes camera/drawing properties
 */
void init()
{
    // Set the Camera Perspective
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(90.0,1.0,0.01,1000.0);

    // Set background color to grey
    glClearColor(0.5, 0.5, 0.5, 0.0);

    // Disable Drawing of BackFace of Polygon
    glCullFace(GL_BACK);
    glEnable(GL_CULL_FACE);

    // Enable Smooth Shading
    glShadeModel(GL_SMOOTH);
    glEnable(GL_POLYGON_SMOOTH);
    glEnable(GL_LINE_SMOOTH);
}

/**
 * reshape - Called every time window is resized
 *           to update the projection matrix, and
 *           to preserve aspect ratio
 */
void reshape(int w, int h) 
{
    // Prevent a divide by zero, when h is zero.
    // You can't make a window of zero height.
    if(h == 0)
    {
        // Set Minimum Height of Window
        h = 1;
    }

    // Setup image size
    glViewport(0, 0, w, h);

    // Reset the coordinate system before modifying
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    // Set the perspective
    double aspectRatio = 1.0 * w / h;
    gluPerspective(60.0f, aspectRatio, 0.01f, 1000.0f);

    // Set back to ModelView
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // Update Member Variables
    _windowWidth = w;
    _windowHeight = h;

    // Double buffer flush
    glutPostRedisplay();
}

/**
 * display
 */
void display()
{
    // Clear buffers
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // camera parameters are Phi, Theta, R
    gluLookAt(R * cos(Phi) * cos (Theta), R * sin(Phi) * cos (Theta), R * sin (Theta),
            0.0,0.0,0.0, 0.0,0.0,1.0);

    /****************************/
    /********* Lighting *********/
    /****************************/
    // You are encouraged to change lighting parameters or
    // make improvements/modifications to the lighting model.
    // This way, you will personalize your assignment and your
    // assignment will stick out.


    // global ambient light
    GLfloat aGa[] = { 0.0, 0.0, 0.0, 0.0 };

    // light 's ambient, diffuse, specular
    GLfloat lKa0[] = { 0.0, 0.0, 0.0, 1.0 };
    GLfloat lKd0[] = { 1.0, 1.0, 1.0, 1.0 };
    GLfloat lKs0[] = { 1.0, 1.0, 1.0, 1.0 };

    GLfloat lKa1[] = { 0.0, 0.0, 0.0, 1.0 };
    GLfloat lKd1[] = { 1.0, 0.0, 0.0, 1.0 };
    GLfloat lKs1[] = { 1.0, 0.0, 0.0, 1.0 };

    GLfloat lKa2[] = { 0.0, 0.0, 0.0, 1.0 };
    GLfloat lKd2[] = { 1.0, 1.0, 0.0, 1.0 };
    GLfloat lKs2[] = { 1.0, 1.0, 0.0, 1.0 };

    GLfloat lKa3[] = { 0.0, 0.0, 0.0, 1.0 };
    GLfloat lKd3[] = { 0.0, 1.0, 1.0, 1.0 };
    GLfloat lKs3[] = { 0.0, 1.0, 1.0, 1.0 };

    GLfloat lKa4[] = { 0.0, 0.0, 0.0, 1.0 };
    GLfloat lKd4[] = { 0.0, 0.0, 1.0, 1.0 };
    GLfloat lKs4[] = { 0.0, 0.0, 1.0, 1.0 };

    GLfloat lKa5[] = { 0.0, 0.0, 0.0, 1.0 };
    GLfloat lKd5[] = { 1.0, 0.0, 1.0, 1.0 };
    GLfloat lKs5[] = { 1.0, 0.0, 1.0, 1.0 };

    GLfloat lKa6[] = { 0.0, 0.0, 0.0, 1.0 };
    GLfloat lKd6[] = { 1.0, 1.0, 1.0, 1.0 };
    GLfloat lKs6[] = { 1.0, 1.0, 1.0, 1.0 };

    GLfloat lKa7[] = { 0.0, 0.0, 0.0, 1.0 };
    GLfloat lKd7[] = { 0.0, 1.0, 1.0, 1.0 };
    GLfloat lKs7[] = { 0.0, 1.0, 1.0, 1.0 };

    // light positions and directions
    GLfloat lP0[] = { -1.999, -1.999, -1.999, 1.0 };
    GLfloat lP1[] = { 1.999, -1.999, -1.999, 1.0 };
    GLfloat lP2[] = { 1.999, 1.999, -1.999, 1.0 };
    GLfloat lP3[] = { -1.999, 1.999, -1.999, 1.0 };
    GLfloat lP4[] = { -1.999, -1.999, 1.999, 1.0 };
    GLfloat lP5[] = { 1.999, -1.999, 1.999, 1.0 };
    GLfloat lP6[] = { 1.999, 1.999, 1.999, 1.0 };
    GLfloat lP7[] = { -1.999, 1.999, 1.999, 1.0 };

    // jelly material color

    GLfloat mKa[] = { 0.0, 0.0, 0.0, 1.0 };
    GLfloat mKd[] = { 0.3, 0.3, 0.3, 1.0 };
    GLfloat mKs[] = { 1.0, 1.0, 1.0, 1.0 };
    GLfloat mKe[] = { 0.0, 0.0, 0.0, 1.0 };

    // Set up lighting
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, aGa);
    glLightModelf(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE);
    glLightModelf(GL_LIGHT_MODEL_TWO_SIDE, GL_FALSE);

    // set up cube color
    glMaterialfv(GL_FRONT, GL_AMBIENT, mKa);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, mKd);
    glMaterialfv(GL_FRONT, GL_SPECULAR, mKs);
    glMaterialfv(GL_FRONT, GL_EMISSION, mKe);
    glMaterialf(GL_FRONT, GL_SHININESS, 60);

    // macro to set up light i
#define LIGHTSETUP(i)\
		glLightfv(GL_LIGHT##i, GL_POSITION, lP##i);\
		glLightfv(GL_LIGHT##i, GL_AMBIENT, lKa##i);\
		glLightfv(GL_LIGHT##i, GL_DIFFUSE, lKd##i);\
		glLightfv(GL_LIGHT##i, GL_SPECULAR, lKs##i);\
		glEnable(GL_LIGHT##i)

    LIGHTSETUP (0);
    LIGHTSETUP (1);
    LIGHTSETUP (2);
    LIGHTSETUP (3);
    LIGHTSETUP (4);
    LIGHTSETUP (5);
    LIGHTSETUP (6);
    LIGHTSETUP (7);

    // Enable lighting
    glEnable(GL_LIGHTING);
    glEnable(GL_DEPTH_TEST);

    // Show the cube
    showCube(&jello);

    // Disable lighting
    glDisable(GL_LIGHTING);

    // Show the bounding box
    showBoundingBox();

    glutSwapBuffers();
}

/**
 * idle
 */
void idle()
{
    char s[20]="picxxxx.ppm";
    int i;

    // save screen to file
    s[3] = 48 + (sprite / 1000);
    s[4] = 48 + (sprite % 1000) / 100;
    s[5] = 48 + (sprite % 100 ) / 10;
    s[6] = 48 + sprite % 10;

    if (saveScreenToFile == 1)
    {
        saveScreenshot(_windowWidth, _windowHeight, s);
        saveScreenToFile=0; // save only once, change this if you want continuos image generation (i.e. animation)
        sprite++;
    }

    // Allow only 300 snapshots
    if (sprite >= 300)
    {
        // Exit Application
        exit(0);
    }

    if (pause == 0)
    {
        // Get Integrator
        string integrator = std::string(jello.integrator);

        // Initialize Integrator Strings
        string rk4 = "RK4";
        string euler = "EULER";

        // Check if Integrator is Euler
        if(integrator.compare(euler) == 0)
        {
            // Peform Euler Integration
            Euler(&jello);
        }
        // Check if Integrator is RK$
        else if(integrator.compare(rk4) == 0)
        {
            // Perform Runge-Katta 4 Integration
            RK4(&jello);
        }
    }

    glutPostRedisplay();
}

/**
 * main - The main function of the Jello Program
 */
int main (int argc, char ** argv)
{
    // Check if less than 2 Arguments
    if (argc < 2)
    {
        // Log Error Statement and Exit Program
        printf ("Oops! You didn't say the jello world file!\n");
        printf ("Usage: %s [worldfile]\n", argv[0]);
        exit(0);
    }

    // Read in Scene from World File
    readWorld(argv[1], &jello);

    // Initialize GLUT
    glutInit(&argc,argv);

    // Request double buffer, color, and depth testing
    glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);

    // Set window size and poistion
    _windowWidth = 640;
    _windowHeight = 480;
    glutInitWindowSize(_windowWidth, _windowHeight);
    glutInitWindowPosition(0,0);

    // Create Window
    glutCreateWindow ("Jello Cube");

    // GLUT Callbacks
    glutDisplayFunc(display);
    glutIdleFunc(idle);
    glutMotionFunc(mouseMotionDrag);
    glutReshapeFunc(reshape);
    glutPassiveMotionFunc(mouseMotion);
    glutMouseFunc(mouseButton);
    glutKeyboardFunc(keyboardFunc);

    // Do Initialization
    init();

    glutMainLoop();
    return(0);
}
