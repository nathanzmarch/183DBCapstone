# Copyright 1996-2020 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Demonstration of inverse kinematics using the "ikpy" Python module."""

try:
    from ikpy.chain import Chain
    from ikpy.link import OriginLink, URDFLink
except ImportError:
    import sys
    sys.exit('The "ikpy" Python module is not installed. '
             'To run this sample, please upgrade "pip" and install ikpy with this command: "pip install ikpy"')

import math
from controller import Supervisor, Camera, CameraRecognitionObject, Node
import numpy as np
import re

# HELPER FUNCTIONS
def makeSquareArrays(x,y,z):
    z=[0.32 for i in range(160)]
    
def calculateDistance2D(x1,y1,x2,y2):  
    dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)  
    return dist  

def calculateDistance3D(x1,y1,x2,y2):  
    dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)  
    return dist  
    
def calculateDist3D(x1,y1,z1,x2,y2,z2):  
    dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2-z1)**2)  
    return dist  
# Create the arm chain.
# The constants below have been manually extracted from the Irb4600-40.proto file, looking at the HingeJoint node fields.
# The chain should contain the "E motor" bone, because this bone defines the hand position.
armChain = Chain(name='arm', links=[
    OriginLink(),
    URDFLink(
        name="A motor",
        bounds=[-3.1415, 3.1415],
        translation_vector=[0, 0, 0.159498],
        orientation=[0, 0, 0],
        rotation=[0, 0, 1]
    ),
    URDFLink(
        name="B motor",
        bounds=[-1.5708, 2.61799],
        translation_vector=[0.178445, -0.122498, 0.334888],
        orientation=[0, 0, 0],
        rotation=[0, 1, 0]
    ),
    URDFLink(
        name="C motor",
        bounds=[-3.1415, 1.309],
        translation_vector=[-0.003447, -0.0267, 1.095594],
        orientation=[0, 0, 0],
        rotation=[0, 1, 0]
    ),
    URDFLink(
        name="D motor",
        bounds=[-6.98132, 6.98132],
        translation_vector=[0.340095, 0.149198, 0.174998],
        orientation=[0, 0, 0],
        rotation=[1, 0, 0]
    ),
    URDFLink(
        name="E motor",
        bounds=[-2.18166, 2.0944],
        translation_vector=[0.929888, 0, 0],
        orientation=[0, 0, 0],
        rotation=[0, 1, 0]
    )
], active_links_mask=[False, True,True,True,True,True])

# Initialize the Webots Supervisor.
supervisor = Supervisor()
timeStep = int(4 * supervisor.getBasicTimeStep())

# Initialize the arm motors.
motors = []
for motorName in ['A motor', 'B motor', 'C motor', 'D motor', 'E motor', 'F motor']:
    motor = supervisor.getMotor(motorName)
    motor.setVelocity(1.0)
    motors.append(motor)

# Get the arm and target nodes.
target = supervisor.getFromDef('TARGET')
arm = supervisor.getFromDef('ARM')
trans_field = arm.getField("translation")
force_node = supervisor.getFromDef('FORCE')
# print(force_node.getField("customData"))
# cam1 = supervisor.getCamera("cam1")
# cam1.enable()
global finishtime
count = 0
startCount = False
stopGetTime = False

Kp = 0.001
maxtheta = 0
mintheta = 3
maxphi = 0
minphi = 3
maxgamma = 0
mingamma = 3
#parse obj
reComp = re.compile("(?<=^)(v |vn |vt |f )(.*)(?=$)", re.MULTILINE)
# CHANGE THIS OBJ FILE IF IMPORTING NEW OBJ
with open("Sphere.obj") as f:
    data = [txt.group() for txt in reComp.finditer(f.read())]

v_arr, vn_arr, vt_arr, f_arr = [], [], [], []
verts = []
indices = []
hashindices = {}
vertices, normals = [], []
actnormals = []
index = 0
for line in data:
    tokens = line.split(' ')
    if tokens[0] == 'v':
        for c in tokens[1:]:
            v_arr.append(float(c))
    elif tokens[0] == 'vn':
        for c in tokens[1:]:
            vn_arr.append(float(c))
    elif tokens[0] == 'vt':
        for c in tokens[1:]:
            vt_arr.append(float(c))
    elif tokens[0] == 'f':
        f_arr.append([[int(i) if len(i) else 0 for i in c.split('/')] for c in tokens[1:]])
        quad = False
        for j in range(1,len(tokens)):
            if(j == 4 and not quad):
                j = 3
                quad = True
            if(tokens[j] in hashindices):
                indices.append(hashindices[tokens[j]])
            else:
                vertex = tokens[j].split('/')
                verts.append(v_arr[(int(vertex[0]) - 1) * 3 + 0])
                verts.append(v_arr[(int(vertex[0]) - 1) * 3 + 1])
                verts.append(v_arr[(int(vertex[0]) - 1) * 3 + 2])
                if(len(vt_arr)):
                    vt_arr.append(vt_arr[( (int(vertex[1]) - 1) or int(vertex[0])) * 2 + 0])
                    vt_arr.append(vt_arr[( (int(vertex[1]) - 1) or int(vertex[0])) * 2 + 1])
                normals.append(vn_arr[( (int(vertex[2]) - 1) or int(vertex[0])) * 3 + 0])
                normals.append(vn_arr[( (int(vertex[2]) - 1) or int(vertex[0])) * 3 + 1])
                normals.append(vn_arr[( (int(vertex[2]) - 1) or int(vertex[0])) * 3 + 2])

                hashindices[tokens[j]] = index
                indices.append(index)
                index += 1
            if(j == 4 and quad):
                indices.append(hashindices[tokens[0]])

for j in range(len(verts)/3):
    vertices.append([verts[3*j+0],verts[3*j+1],verts[3*j+2] ])
for j in range(len(normals)/3):
    actnormals.append([normals[3*j+0],normals[3*j+1],normals[3*j+2]])
    
#Verify that these are equal to guarantee correct parsing
print("vert len: ", len(vertices))
print("norm len: ", len(actnormals))
# 0 = No drawing
# 1 = square on paper
# 2 = circle on paper
# 3 = line on paper
# 4 = arc on tube (inverse_kinematics_tube)
# 5 = tattoo on table (inverse_kinematics_tattoo_2d)
# 6 = Cross on cylinder
# 7 = Circle on cylinder
goal = 10
noise = 0

# Coordinates for a square on paper
if goal == 1:
    arr_size = 160
    x_arr = [0.71 for i in range(arr_size/4)]
    x_arr.extend([0.02*i + 0.7 for i in range(arr_size/4)])
    x_arr.extend([1.5 for i in range(arr_size/4)])
    x_arr.extend([1.5 - 0.02*i  for i in range(arr_size/4)])
    y_arr = [0.02*i - 1.35 for i in range(arr_size/4)]
    y_arr.extend([-0.57 for i in range(arr_size/4)])
    y_arr.extend([-0.55 - 0.02*i  for i in range(arr_size/4)])
    y_arr.extend([-1.35 for i in range(arr_size/4)])
    z_arr = [0.34 for i in range(arr_size)]

# Coordinates for a circle on paper
if goal == 2:
    angles = np.arange(0, 6.28, 0.01)
    arr_size = len(angles)
    
    x_arr = 0.25 * np.sin(angles) + 1.1
    y_arr = 0.25 * np.cos(angles) - 0.95
    z_arr = 0.31 + 0 * angles

# Coordinates for a line on inclined plane
if goal == 3:
    arr_size = 80
    x_arr = [0.01*i + 0.7 for i in range(arr_size)]
    y_arr = [0.01*i - 1.35 for i in range(arr_size)]
    z_arr = [0.01*i + 0.32 for i in range(arr_size)]

# Coordinates for an arc on a tube (load tube world)
if goal == 4:
    angles = np.arange(np.pi/6, 5*np.pi/6, 0.01)
    x_arr = 0.5*np.cos(angles) + 1.2
    y_arr = 0 * angles - 1.03
    z_arr = 0.5*np.sin(angles) + 0.138 # Adjust offset as necessary
    
    angles = np.arange(0, 0.5, 0.01)
    y_arr2 = y_arr[-1] - angles
    x_arr2 = x_arr[-1] + 0 * angles
    z_arr2 = z_arr[-1] + 0 * angles
    x_arr = np.concatenate((x_arr, x_arr2), 0)
    y_arr = np.concatenate((y_arr, y_arr2), 0)
    z_arr = np.concatenate((z_arr, z_arr2), 0)
    
    angles = np.arange(np.pi/6, 5*np.pi/6, 0.01)
    angles = np.pi - angles;
    x_arr2 = 0.5*np.cos(angles) + 1.2
    y_arr2 = 0 * angles - 1.53
    z_arr2 = 0.5*np.sin(angles) + 0.138 # Adjust offset as necessary
    x_arr = np.concatenate((x_arr, x_arr2), 0)
    y_arr = np.concatenate((y_arr, y_arr2), 0)
    z_arr = np.concatenate((z_arr, z_arr2), 0)
    
    angles = np.arange(0, 0.5, 0.01)
    y_arr2 = y_arr[-1] + angles
    x_arr2 = x_arr[-1] + 0 * angles
    z_arr2 = z_arr[-1] + 0 * angles + 0.02
    x_arr = np.concatenate((x_arr, x_arr2), 0)
    y_arr = np.concatenate((y_arr, y_arr2), 0)
    z_arr = np.concatenate((z_arr, z_arr2), 0)
    
    arr_size = len(x_arr)

# Coordinates for a tattoo (load 2D tattoo world)
if goal == 5:
    scale_ratio = 0.106282
    shifts = [1.005392, 0.192004, 0]
    tattoo = np.loadtxt("flatTattoo.txt")
    tattoo_x = tattoo[:, 0] * scale_ratio + shifts[0]
    tattoo_y = tattoo[:, 2] * scale_ratio + shifts[1]
    tattoo_z = tattoo[:, 1] * scale_ratio + shifts[2]
    
    # PARAMETERS
    margin = 0.1     # Must find this by plotting the frequency of point-to-point distances
    z_lift = 1       # How much to lift up after finishing a stroke
    
    # ONLY FOR FLAT SURFACE
    height = 0.39  # Adjust as necessary
    tattoo_z = tattoo_z * 0 + height
    
    # Loops through all points, and adds commands to terminate and start new strokes 
    # when it detects a new stroke is about to begin using point distance. 
    x_arr_list = [tattoo_x[0]]
    y_arr_list = [tattoo_y[0]]
    z_arr_list = [tattoo_z[0]]
    for i in range(0, len(tattoo_x)):
        x = tattoo_x[i]
        y = tattoo_y[i]
        z = tattoo_z[i]
        
        x0 = x_arr_list[-1]
        y0 = y_arr_list[-1]
        z0 = z_arr_list[-1]
        
        # Add a jump command if the new point is far away from the old point
        if calculateDistance2D(x0, y0, x, y) > margin:
            x_arr_list.extend([ x0, x , x ])
            y_arr_list.extend([ y0, y , y ])
            z_arr_list.extend([ z0 + z_lift, z0 + z_lift, z ])
        else:
            x_arr_list.append(x)
            y_arr_list.append(y)
            z_arr_list.append(z)
            
    # Avoids collision
    x_arr_list.append(x)
    y_arr_list.append(y)
    z_arr_list.append(z + 2)
    
    # Convert lists to np arrays (lists were used as it is more efficient to modify lists)
    x_arr = np.array(x_arr_list)
    y_arr = np.array(y_arr_list)       
    z_arr = np.array(z_arr_list)

    arr_size = len(x_arr)
    
if goal >= 6:
    if goal == 6:
        tattoo = np.loadtxt("Line_Curved.txt")
        decimation = 16
    elif goal == 7:
        tattoo = np.loadtxt("Line_Straight.txt")
        decimation = 16
    elif goal == 8:
        tattoo = np.loadtxt("Circle_Curved.txt")
        decimation = 40
    elif goal == 9:
        tattoo = np.loadtxt("Tattoo_Curved.txt")
        decimation = 16;
    elif goal == 10:
        tattoo = np.loadtxt("Circle_On_Sphere.txt");
        decimation = 1
        
    tattoo = tattoo[1::decimation]
    
    scale_ratio = 1
    shifts = [0, 0, 0.04]
    tattoo_x = tattoo[:, 0] * scale_ratio + shifts[0]
    tattoo_y = tattoo[:, 2] * scale_ratio + shifts[1]
    tattoo_z = tattoo[:, 1] * scale_ratio + shifts[2]
    
    # PARAMETERS
    margin = 0.1     # Must find this by plotting the frequency of point-to-point distances
    z_lift = 1.5       # How much to lift up after finishing a stroke
    
    # ONLY FOR FLAT SURFACE
    #height = 0.39  # Adjust as necessary
    #tattoo_z = tattoo_z * 0 + height
    
    # Loops through all points, and adds commands to terminate and start new strokes 
    # when it detects a new stroke is about to begin using point distance. 
    x_arr_list = [tattoo_x[0]]
    y_arr_list = [tattoo_y[0]]
    z_arr_list = [tattoo_z[0]]
    for i in range(0, len(tattoo_x)):
    
        
        x = tattoo_x[i]
        y = tattoo_y[i]
        z = tattoo_z[i]
        
        x0 = x_arr_list[-1]
        y0 = y_arr_list[-1]
        z0 = z_arr_list[-1]
        
        # Add a jump command if the new point is far away from the old point
        if calculateDistance2D(x0, y0, x, y) > margin:
            x_arr_list.extend([ x0, x , x ])
            y_arr_list.extend([ y0, y , y ])
            z_arr_list.extend([ z0 + z_lift, z0 + z_lift, z ])
        else:
            x_arr_list.append(x)
            y_arr_list.append(y)
            z_arr_list.append(z)
            
    # Avoids collision
    x_arr_list.append(tattoo_x[-1])
    y_arr_list.append(tattoo_y[-1])
    z_arr_list.append(tattoo_z[-1] + 1)
    
    # Convert lists to np arrays (lists were used as it is more efficient to modify lists)
    x_arr = np.array(x_arr_list)
    y_arr = np.array(y_arr_list)       
    z_arr = np.array(z_arr_list)

    arr_size = len(x_arr)


dist = []
# Begin Drawing
while supervisor.step(timeStep) != -1 and goal > 0:

    x = x_arr[count]
    y = y_arr[count]
    z = z_arr[count]
    if count  == arr_size-1:
        count = 0
        stopGetTime = True
    if startCount:
        count = count + 1
    if not stopGetTime:
        finishtime = supervisor.getTime()
    
    # Apply noise
    if noise == 1: 
        noise = np.random.normal(0, 0.01, 3)
        x += noise[0]
        y += noise[1]
        z += noise[2]

    # Call "ikpy" to compute the inverse kinematics of the arm.
    ikResults = armChain.inverse_kinematics([
        [1, 0, 0, x],
        [0, 1, 0, y],
        [0, 0, 1, z],
        [0, 0, 0, 1]
    ])
    # Actuate the 3 first arm motors with the IK results.
    for i in range(3):
        motors[i].setPosition(ikResults[i + 1])
    
    min = 50
    index = 0
    for i in range(len(vertices)):
        temp = calculateDist3D(vertices[i][0],vertices[i][1],vertices[i][2],x,z,y)
        if ( temp < min):
            min = temp
            index = i
        if i == len(vertices) - 1:
            dist.append(min)
    normal = actnormals[index]
    
    phi = np.arccos(normal[0])
    gamma = np.arccos(normal[1])
    if normal[2] > 0 and normal[0] > 0:
        theta = np.arctan(abs(normal[2]/normal[0]))
    if normal[2] > 0 and normal[0] < 0:
        theta = math.pi - np.arctan(abs(normal[2]/normal[0])) 
    if normal[2] < 0 and normal[0] < 0:
        theta = math.pi + np.arctan(abs(normal[2]/normal[0]))
    if normal[2] < 0 and normal[0] > 0:
        theta = 2 * math.pi - np.arctan(abs(normal[2]/normal[0]))
    
    if(theta > maxtheta):
        maxtheta = theta
    if(theta < mintheta):
        mintheta = theta
    if(phi > maxphi):
        maxphi = phi
    if(phi < minphi):
        minphi = phi
    if(gamma > maxgamma):
        maxgamma = gamma
    if(gamma < mingamma):
        mingamma = gamma
    
    print("index:", index)
    print("vertex:", vertices[index])
    print("Normal:", actnormals[index])
    
    print("phi: %g min: %g max : %g" % (phi,minphi,maxphi));
    print("gamma: %g min: %g max : %g" % (gamma,mingamma,maxgamma));
    print("theta: %g min: %g max : %g" % (theta,mintheta,maxtheta));
    
    print("dist", min)
   
    
    
    # ORIGINAL KEEP HAND DOWN AND PERPENDICULAR
    # motors[4].setPosition(-ikResults[2] - ikResults[3] + math.pi / 2)
    # motors[5].setPosition(ikResults[1])
    
    #New Perpendicular calculation
    motor4 = - ikResults[2] - ikResults[3] + math.pi/2 - gamma
    print("Motor 3: %g"%(theta))
    print("Motor 4: %g"%(motor4))
    if( motor4 > 2.0944):
        motor4 = 2.0944
    motors[4].setPosition(motor4)
    #TODOL orient the motor 5 so more inline with surface
    motors[3].setPosition(theta)
    # Report Position
    # values = trans_field.getSFVec3f()
    print("ARM drawing is at position: %g %g %g" % (x, z, y))
    print("Percent Complete %g" % (float(count)/arr_size * 100))
    forcestr = force_node.getField("customData").getSFString()
    force = float(forcestr)
    print("Touch sensor experiencing force of %gN" % (force))
    # if(count < arr_size - 2):
        # error = force - 100
        # output = Kp * error
        
        # x_arr[count+1] += output*normal[0]
        # y_arr[count+1] += output*normal[2]
        # z_arr[count+1] += output*normal[1]
        # x_arr[count+1] += normal[0]
        # y_arr[count+1] += normal[2]
        # z_arr[count+1] += normal[1]
        # count -= 1
    print("Corrected coords:  %g %g %g" % (x_arr[count], z_arr[count], y_arr[count]) )
    # Conditions to start/stop drawing and leave this loop.
    # if supervisor.getTime() > 2 * math.pi + 1.5:
    if supervisor.getTime() > finishtime + 0.1:
        break
    elif supervisor.getTime() > 1.5:
        # Note: start to draw at 1.5 second to be sure the arm is well located.
        supervisor.getPen('pen').write(True)
        startCount = True

# print(dist)
# sum = 0
# for i in range(len(dist)):
    # sum += dist[i]
# average = sum / len(dist)
# print("average", average)
    
# Loop 2: Move the arm hand to the target.
print('Move the yellow and black sphere to move the arm...')
while supervisor.step(timeStep) != -1:
    # Get the absolute postion of the target and the arm base.
    targetPosition = target.getPosition()
    armPosition = arm.getPosition()

    # Compute the position of the target relatively to the arm.
    # x and y axis are inverted because the arm is not aligned with the Webots global axes.
    print("Arm position:", armPosition)
    x = targetPosition[0] - armPosition[0]
    y = - (targetPosition[2] - armPosition[2])
    z = targetPosition[1] - armPosition[1]
    values = trans_field.getSFVec3f()
    # print("ARM is at position: %g %g %g" % (values[0], values[1], values[2]))
    # Call "ikpy" to compute the inverse kinematics of the arm.
    ikResults = armChain.inverse_kinematics([
        [1, 0, 0, x],
        [0, 1, 0, y],
        [0, 0, 1, z],
        [0, 0, 0, 1]
    ])

    # Actuate the 3 first arm motors with the IK results.
    # for i in range(3):
        # motors[i].setPosition(ikResults[i + 1])
print("Printing Complete")    
    # Report Position
    # values = trans_field.getSFVec3f()
    # print("ARM is at position: %g %g %g" % (x, y, z))

# while supervisor.step(TIME_STEP) != -1:
    # values = trans_field.getSFVec3f()
    # print("ARM is at position: %g %g %g" % (values[0], values[1], values[2]))