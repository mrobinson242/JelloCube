================================================================
CSCI 520, Assignment 1

Name:  Matthew Robinson
Id:    9801107811
Email: robi973@usc.edu
OS:    macOS Mojave
================================================================

======================== Description ===========================
In this Assignment, I have simulated a Jello Cube using
a Mass-Spring System. The System is modeled by 8 * 8 * 8
(512) discrete mass points all of equal mass.

The System is Composed of three types of Springs.
Structural, Shear, and Bend Springs. The different types
of springs allow for a stable configuration of the mass
spring network, allowing for an accurate simulation of
the Jello Material.

The program functions by reading in information about the
initial position of the cube, initial velocity, and various
other simulation parameters from a "world" file on the disk.

Based on these initial parameters and the specified Integration
method, the program calculates the forces acting on each
spring and returns an acceleration based on Newton's
Second Law (F=ma); The Integration Methods, then take the
calculated acceleration and models the movement of the cube
at the specified timestep in the "world" file.

I have also implemented Collision Detection in the program.
There is a defined Bounding Box (-2 to 2) that prevents the cube
from escaping and correctly handles the response of the Jello
Cube hitting any of the six walls.

The Program also supports an external non-homogeneous
time-independent External Force Field. The force field
can be provided in the "world" file as an array of 3D
force vectors at a node of a 3D Discrete Grid. These
force vectors can additionally affect the cube, and 
is included in the summation of forces used to 
calculate Newton's Second Law.

Lastly, the OpenGL Lighting Model has been coded with a 
combination of Blue and Yellow Lights. The Lighting Combination
gives the Jello Cube a varying Greenish appearence. The Goal
of this as to give the appearence of the typical green jello
found in Hospitals
================================================================

====================== Compiling ===============================
> unzip JelloCube.zip in Desktop
> cd ~/Desktop/JelloCube
> make
> ./jello world/<World File>
================================================================

============================ Inputs ============================
ESC: exit application
v: switch wireframe/triangle mode
s: display structureal springs on/off
h: display shear springs on/off
b: display bend springs on/off
space: save the current screen to a file
p: pause on/off
z: camera zoom in
x: camera zoom out
right mouse button + mvoe mouse: camera control
e: reset camera to default position
================================================================

========================= Extra Credit =========================
N/A
================================================================
