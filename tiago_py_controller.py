#Python based Tiago Controller
from controller import Robot, Motor, PositionSensor
import math
import csv
import pandas as pd
import operator
import numpy

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# setup for the parts.
part_names = ["head_1_joint", "head_2_joint","torso_lift_joint",  "arm_right_1_joint", "arm_right_2_joint",
         "arm_right_3_joint", "arm_right_4_joint", "arm_right_5_joint", "arm_right_6_joint",
         "arm_right_7_joint", "arm_left_1_joint",  "arm_left_2_joint","arm_left_3_joint",
         "arm_left_4_joint",  "arm_left_5_joint",  "arm_left_6_joint","arm_left_7_joint",
         "wheel_left_joint",  "wheel_right_joint"];
         
# Defaults      
target_pos = [0.0, 0.0, 0.15, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
              0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];             
current_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
              0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
# Enable(1) or disable(0) certain motors in the robot by setting their velocity to zero.               
# motorMask = [1, 1, 0, 0, 1, 0,1, 0, 0, 0, 0,1, 0,1, 0, 0, 0, 0, 0]            
robot_parts=[]


for i in range(len(part_names)):
    robot_parts.append(robot.getDevice(part_names[i]))
    # sensor = PositionSensor(robot.getDevice(part_names[0]))
    robot_parts[i].setPosition(float(target_pos[i]))
    robot_parts[i].setVelocity(robot_parts[i].getMaxVelocity() / 2.0)
    

   
# open the csv file
with open(r"movements.csv") as csv_file: 
      
    # read the csv file
    csv_reader = csv.reader(csv_file, delimiter=']')
       
    # create a pandas dataframe from the csv
    df = pd.DataFrame([csv_reader], index=None)
    df.head()


stepList = []

# Create the list of target positions from the pandas dataframe
for val in list(df[0]):
    for i in range(len(val)-1):
        step = val[i].split(',')
        for j in range(len(step)):
            step[j] = step[j].replace('[','')
        if(step[0] == ''):
            step = step[1:]
        stepList.append(step)
stepList.append([0,0,math.pi,0,0,0])
    
    
j = 0
# Main loop:
while robot.step(timestep) != -1 and j < len(stepList):
    sum = 0        
    # print(target_pos, current_pos)
    for i in range(len(target_pos)):
        if(float(target_pos[i]) > robot_parts[i].getMaxPosition()):
            target_pos[i] = robot_parts[i].getMaxPosition()
        elif(float(target_pos[i]) < robot_parts[i].getMinPosition()):
            target_pos[i] = robot_parts[i].getMinPosition()
        else:
            target_pos[i] = float(target_pos[i])
        sum += abs(float(target_pos[i]) - float(current_pos[i]))

    if(sum < .3):
        # #back to T-pose
        if(j == len(stepList)-2):
            j = 100;
        # next frame step
        rs = stepList[j][0] # right shoulder
        re = stepList[j][1] # right elbow
        ls = stepList[j][2] # left shoulder
        le = stepList[j][3] # left elbow
        he = stepList[j][4] # head elevation
        ha = stepList[j][5] # head angle
        # print(stepList[j])i
        target_pos = [float(ha), float(he), 0.15, 0.0, math.pi-float(ls), 0.0, float(le), 0.0, 0.0, 0.0,
                  0.0, float(rs), 0.0, -float(re), 0.0, 0.0, 0.0, 0.0, 0.0];
        j+=1
    
    for i in range(len(part_names)):
        if(float(target_pos[i])- current_pos[i] < -0.01):    
            # print(float(target_pos[i]), robot_parts[i].getMinPosition(), "Max:", max(float(target_pos[i]), robot_parts[i].getMinPosition()))
            robot_parts[i].setVelocity(robot_parts[i].getMaxVelocity() / 2.0)
            robot_parts[i].setPosition(max(float(target_pos[i]), robot_parts[i].getMinPosition()))
            current_pos[i] -= robot_parts[i].getVelocity()*(timestep/1000)
        elif(float(target_pos[i]) - current_pos[i] > 0.01):
            robot_parts[i].setVelocity(robot_parts[i].getMaxVelocity() / 2.0)
            robot_parts[i].setPosition(min(float(target_pos[i]), robot_parts[i].getMaxPosition()))
            current_pos[i] += robot_parts[i].getVelocity()*(timestep/1000)
        else:
            current_pos[i] += 0
            robot_parts[i].setVelocity(0)
            
        
    print("CP", j )
 
        

# Enter here exit cleanup code.
