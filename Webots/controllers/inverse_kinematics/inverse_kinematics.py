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
from controller import Supervisor, Camera, CameraRecognitionObject
import numpy as np

def makeSquareArrays(x,y,z):
    z=[0.32 for i in range(160)]
    # x = [
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
])

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

# cam1 = supervisor.getCamera("cam1")
# cam1.enable()
# Loop 1: Draw a circle on the paper sheet.
print('Draw a circle on the paper sheet...')
global finishtime
count = 0
startCount = False
stopGetTime = False
# draw line
arr_size = 80
x_arr = [0.01*i + 0.7 for i in range(arr_size)]
y_arr = [0.01*i - 1.35 for i in range(arr_size)]
z_arr = [0.0092*i + 0.32 for i in range(arr_size)]
# draw 2-d square
# arr_size = 160
# x_arr = [0.71 for i in range(arr_size/4)]
# x_arr.extend([0.02*i + 0.7 for i in range(arr_size/4)])
# x_arr.extend([1.5 for i in range(arr_size/4)])
# x_arr.extend([1.5 - 0.02*i  for i in range(arr_size/4)])
# y_arr = [0.02*i - 1.35 for i in range(arr_size/4)]
# y_arr.extend([-0.57 for i in range(arr_size/4)])
# y_arr.extend([-0.55 - 0.02*i  for i in range(arr_size/4)])
# y_arr.extend([-1.35 for i in range(arr_size/4)])
# z_arr = [0.34 for i in range(arr_size)]

while supervisor.step(timeStep) != -1:
    t = supervisor.getTime()

    # Use the circle equation relatively to the arm base as an input of the IK algorithm.
    # x = 0.25 * math.sin(t) + 1.1
    # y = 0.25 * math.sin(t) - 0.95
    # z = 0.32
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
    noise = np.random.normal(0, 0.01, 3)
    #Apply Noise
    # x += noise[0]
    # y += noise[1]
    # z += noise[2]

    # Call "ikpy" to compute the inverse kinematics of the arm.
    ikResults = armChain.inverse_kinematics([
        [1, 0, 0, x],
        [0, 1, 0, y],
        [0, 0, 1, z],
        [0, 0, 0, 1]
    ])
    values = trans_field.getSFVec3f()
    # print("ARM is at position: %g %g %g" % (values[0], values[1], values[2]))
    # Actuate the 3 first arm motors with the IK results.
    for i in range(3):
        motors[i].setPosition(ikResults[i + 1])
    # Keep the hand orientation down.
    motors[4].setPosition(-ikResults[2] - ikResults[3] + math.pi / 2)
    # Keep the hand orientation perpendicular.
    motors[5].setPosition(ikResults[1])

    # Conditions to start/stop drawing and leave this loop.
    # if supervisor.getTime() > 2 * math.pi + 1.5:
    if supervisor.getTime() > finishtime + 0.1:
        break
    elif supervisor.getTime() > 1.5:
        # Note: start to draw at 1.5 second to be sure the arm is well located.
        supervisor.getPen('pen').write(True)
        startCount = True
# Loop 2: Move the arm hand to the target.
print('Move the yellow and black sphere to move the arm...')
while supervisor.step(timeStep) != -1:
    # Get the absolute postion of the target and the arm base.
    targetPosition = target.getPosition()
    armPosition = arm.getPosition()

    # Compute the position of the target relatively to the arm.
    # x and y axis are inverted because the arm is not aligned with the Webots global axes.
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
    for i in range(3):
        motors[i].setPosition(ikResults[i + 1])

while supervisor.step(TIME_STEP) != -1:
    values = trans_field.getSFVec3f()
    # print("ARM is at position: %g %g %g" % (values[0], values[1], values[2]))