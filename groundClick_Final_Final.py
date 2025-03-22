#!/usr/bin/env python
import rospy
import numpy as np
import cv2
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from arena import *

scene = Scene(host="arenaxr.org", scene="groundClick", namespace="matthewkibarian")

class Robot:
    def __init__(self):
        self.position = [0,0,0] #initial position of robot (with offset)
        self.rotation = [0,0,0]
        self.model = GLTF(object_id="robot", position=self.position, rotation=self.rotation, scale=(0.25,0.25,0.25), url="store/models/Duck.glb", persist=True)
        self.targetPos = [0,0,0]
    def addModel(self): #add model to ARENA
        scene.add_object(self.model)
    def update(self, x, z, rotation):
        self.position[0] = -x + 3.5 #offset
        self.position[2] = z - 14.75 #offset

        wr = rotation[3]
        xr = rotation[0]
        yr = rotation[1]
        zr = rotation[2]
        
        roll = np.arctan2(2 * (wr * xr + yr * zr), 1 - 2 * (xr**2 + yr**2))
        
        sin_pitch = 2 * (wr * yr - zr * xr)
        if np.abs(sin_pitch) >= 1:
            pitch = np.sign(sin_pitch) * np.pi / 2
        else:
            pitch = np.arcsin(sin_pitch)
        
        yaw = np.arctan2(2 * (wr * zr + xr * yr), 1 - 2 * (yr**2 + zr**2))
        
        roll = np.degrees(roll)
        pitch = np.degrees(pitch)
        yaw = np.degrees(yaw)

        self.rotation[0] = roll
        self.rotation[1] = yaw + 170
        self.rotation[2] = pitch
        print(self.position)
        self.model.update_attributes(position=(self.position[0],self.position[1],self.position[2]), rotation=(self.rotation[0],self.rotation[1],self.rotation[2]))
        scene.update_object(self.model)
    def update_target(self, x, z):
        self.targetPos[0] = x - 1
        self.targetPos[2] = -z
        print(self.targetPos)
        print("target updated")

robot = Robot() #create the robot
floorplane = Plane(object_id="floorplane", position=(0,0,0), rotation=(270,0,0), color=(0,0,0), scale=(20,20,20), persist=True, opacity = 0.1)
goal = Sphere(object_id="goal", position=(0,-3,0), scale=(0.05,0.05,0.05), color=(255,255,255)) #create a sphere for the goal point
box = Box(object_id='box', position=(0,0,0), color=(255,255,255)) 
arena_map = GLTF(object_id="map", position=(5.75,0,-6.625), rotation=(0,0,0), scale=(1,1,1), url="store/users/matthewkibarian/models/CIC-annex-open-v1.glb", persist=True)
path = Line(object_id="path", start=Position(0,0,0),end=Position(0.1,0.1,0.1),color=(0,0,0)) #show path "as the crow flies"

rpos = [0,0,0] 
global sf 
sf = 100 
rrot = [0,0,0,0] 

global goal_pub
goal_pub = None

def callback1(data): #callback for the robot's position
    global rpos
    global rrot
    rpos_x = float(data.pose.pose.position.x) #get the robot's position and rotation
    rpos_z = float(data.pose.pose.position.y)
    rpos_y = float(data.pose.pose.position.z)
    rrot = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]
    rpos = [rpos_x, rpos_y, rpos_z]
    print(rpos)

def callback2(data): #callback for the laser scan
    ranges = np.array(data.ranges)
    intensities = np.array(data.intensities) #get the ranges and intensities
    angle_inc = data.angle_increment #get the angle increment
    thetas = np.arange(0, ranges.size) * angle_inc  #make array of each angle
    xes = np.cos(thetas) * ranges #calculate x and y coordinates
    ys = np.sin(thetas) * ranges
    for i in range(0, xes.size, 5): #draw the laser scan
        x = xes[i]
        z = ys[i]
        if f"line{i}" in scene.all_objects: #if the line already exists, update it
            line = scene.all_objects[f"line{i}"]
        else: #if the line doesn't exist, create it
            line = Line(object_id=f"line{i}")
        scene.update_object(line, start=(0,0.25,0),end=(x,0.25, z),color=(intensities[i]/2.55,intensities[i],0), parent="robot") #update the lines

def callback3(): #callback for the goal point
    msg = PoseStamped() #create a PoseStamped message
    msg.pose.position.x = robot.targetPos[0] #set the position and orientation
    msg.pose.position.y = robot.targetPos[2]
    msg.pose.position.z = 0
    msg.pose.orientation.z = 0.898
    msg.pose.orientation.w = 0.439
    msg.header.frame_id = "map" #set the frame id
    goal_pub.publish(msg) #publish the message

@scene.run_once
def box1():
    scene.add_object(box)

@scene.run_forever(interval_ms=100)
def update_robot():
    global rrot
    robot.update(rpos[0], rpos[2], rrot) #update the robot's position and rotation every 50ms

def mouseHandler(scene, evt, msg):
    if evt.type =="mousedown": #check if click
        print(evt.data.position.x, evt.data.position.z)
        robot.update_target(evt.data.position.x, evt.data.position.z) #update the target position
        callback3() #run nav callback
        goal.update_attributes(position=(robot.targetPos[0]+1, 0, -robot.targetPos[2]), color=(0,255,0)) #update goal position + color
        scene.update_object(goal)

@scene.run_once
def main(): #add all arena objects
    robot.addModel()
    scene.add_object(goal)
    scene.add_object(floorplane)
    scene.add_object(path)
    scene.add_object(arena_map)

if __name__ == '__main__':
    rospy.init_node('turtlebot', anonymous=True) #initialize the ROS node
    rospy.Subscriber('/odom', Odometry, callback1, queue_size=1000) #subscribe to the robot's position and laser scan
    rospy.Subscriber('/scan', LaserScan, callback2, queue_size=1000) 
    goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10) #create a publisher for the goal point
    #rospy.spin() #keep the node running
    path.update_attributes(start=Position(robot.position[0],0.01,robot.position[2]), end=Position(robot.targetPos[0], 0.01, robot.targetPos[2]), color=(100,0,255))
    scene.update_object(path) #update line path + color in the loop so the line disappears as the robot drives over it
    scene.update_object(floorplane, clickable=True, evt_handler=mouseHandler)
    scene.run_tasks()