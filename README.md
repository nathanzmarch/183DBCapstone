# 183DBCapstone: Team Poseidon

## Description:
We use Webots to simulate a robotic arm drawing images on different objects. We look at the object normals to calculate the orientation of the pen and use inverse kinematics (ikpy) to calculate the position of the arm, and a pressure sensor to determine the force applied to the object being drawn on. 

## Assumptions:
Tattoo and canvas are both small enough for the robot arm to complete the task without having to move the base or rotate the canvas.

## Hyperparameters
Goal: In inverse_kinematics.py change the goal number to draw different images on different surfaces. The image and surfaces should be noted by the goal number.

## Set Up Instructions
### Installation
#### Download Webots
Install ikpy package pip install ikpy

Download Blender 2.7.9b

Install Import/Export to Webots addon

### Input Preparation
Prepare an OBJ file representing a surface

Prepare an SVG file representing a tattoo

### Import Objects to Webots / Open premade world file
Import OBJ file into Blender

Export OBJ file as a wbt file

Open the wbt file, then export the surface as an wbo file

Import the wbo file into the provided world (INSERT_NAME_HERE)

Position the object so it is centered in front of the arm.

If the object is too large, scale down the object in Blender and repeat previous steps - b, c, d.

Make sure that the obj file being read is the one you are drawing on in inverse_kinematics.py

### Webots/Blender Alignment
Given the chosen position of the surface in Webots, change the position of the surface in Blender to align the two.
Variables
X1, Y1, Z1 = Webots Coordinates (Y is up)
X2, Y2, Z2 = Blender Coordinates (Z is up)
Equations

X2 = X1

Y2 = Z1 - (0.616 - 0.23)

Z2 = Y1 - 4.84

### Tattoo Projection
Import the SVG into the Blender project containing the aligned surface. If it is not already, convert the SVG into a mesh.

Position the tattoo so that it is centered right above the spot on the surface that you want to draw on. 

Apply two modifiers to the tattoo

Subdivision Surface

Layout → Modifier Properties → Add Modifier → Subdivision Surface → Increase “Viewport” until smooth.

Increase the “View” value until the mesh has a high number of vertices. To determine this, check the “Verts” value before and after each increase, and stop increasing the “View” value after the number of vertices has gone up by 50,000.

Click “Apply”

Shrinkwrap

Layout → Modifier Properties → Add Modifier

Click “Target” and select the surface you are projecting on

Click “Apply”

 For bigger designs that wrap around - before ii use Add Modifier → SimpleDeform → Bend → Choose appropriative Vertex Group and Axis of origin that does not deform the tattoo → Deform Angle: set to desired curvature.
 
Export the curved tattoo to create commands

Delete the surface so that the tattoo is the only object. This step is not critical, but it makes the process easier, and can be undone with CTRL+Z.

File → Export the tattoo as an OBJ

Change the extension to “.txt”

Delete all lines that don’t start with “v”

Use Find and Replace to replace “v “ with a blank space.

Move this file into the same folder as the inverse_kinematics.py controller, and change the code as instructed in the comments to import the file.
