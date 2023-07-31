#!/usr/bin/env python
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from arena import *

np.set_printoptions(threshold=np.inf)
np.set_printoptions(linewidth=np.inf)
scene = Scene(host="arenaxr.org", scene="groundClick", namespace="matthewkibarian")
class Robot:
    def __init__(self):
        self.position = [0,0,0]
        self.rotation = [0,0,0]
        self.model = GLTF(object_id="robot", position=self.position, rotation=self.rotation, scale=(0.25,0.25,0.25), url="store/models/Duck.glb", persist=True)
        self.targetPos = [0,0,0]
    def addModel(self): #add model to ARENA
        scene.add_object(self.model)
    def update(self, x, z):
        self.position[0] = x
        self.position[2] = z

box = Box(object_id='box', position=(0,0,0), color=(255,255,255))
lidarMap = None
rpos = None

def callback0(data):
    global startTime
    global scene
    global box
    global lidarMap
    lidarMapRaw=np.array(data.data)
    width = data.info.width
    height = data.info.height
    lidarMap = lidarMapRaw.reshape((height, width))

def callback1(data):
    global rpos
    rpos_x = data.pose.pose.position.x
    rpos_y = data.pose.pose.position.y
    rpos_z = data.pose.pose.position.z
    rpos = [rpos_x, rpos_y, rpos_z]
    print(rpos_x,rpos_y,rpos_z)

@scene.run_once
def box1():
    scene.add_object(box)

@scene.run_forever(interval_ms=2000)
def arenaLoop():
    print(np.matrix((lidarMap >= 50) * 1))
    if lidarMap is None:
        return
    for i in range(lidarMap.shape[1]):
        for j in range(lidarMap.shape[0]):
            if lidarMap[j,i] >= 50:
                x = lidarMap.shape[0]-j-(lidarMap.shape[0]/2)
                z = i-(lidarMap.shape[1]/2)
                sf = 4 #scale factor for pos/scale
                if f"box{j}{i}" in scene.all_objects:
                    scene.update_object(scene.all_objects[f"box{j}{i}"], position=(x/sf,3/2,z/sf), color=(0,int(lidarMap[j,i] *2.55),0), scale=(1/sf, 3, 1/sf))
                else:
                    scene.update_object(Box(object_id=f"box{j}{i}", position=(x/sf,3/2,z/sf), color=(0,int(lidarMap[j,i] *2.55),0), scale=(1/sf, 3, 1/sf)))
            else:
                if f"box{j}{i}" in scene.all_objects:
                    scene.delete_object(scene.all_objects[f"box{j}{i}"])
@scene.run_forever(interval_ms=50)
def update_robot():
    robot.update(rpos[0], rpos[2])

def map_listener():
    rospy.init_node('listener0', anonymous=True)
    rospy.Subscriber("/move_base/local_costmap/costmap", OccupancyGrid, callback0)
    scene.run_tasks()
def rpos_listener():
    rospy.init_node('listener1', anonymous=True)
    rospy.Subscriber('/odom', Odometry, callback1)
def talker(x, z):
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    while not rospy.is_shutdown():
        x = robot.targetPos[0]
        z = robot.targetPos[2]
        pub.publish(x, 0, z)


if __name__ == '__main__':
    map_listener()
    rpos_listener()


robot = Robot() #create the robot
floorplane = Image(object_id="floorplane", position=(0,0.01,0), rotation=(270,0,0), scale=(8,4,8), persist=True, opacity=0.35, url="store/users/matthewkibarian/images/map.png")
goal = Sphere(object_id="goal", position=(0,-3,0), scale=(0.05,0.05,0.05), color=(255,255,255)) #create a sphere for the goal point
path = Line(object_id="path", start=Position(0,0,0),end=Position(0.1,0.1,0.1),color=(0,0,0)) #show path that robot will take

@scene.run_once
def main():
    robot.addModel()
    scene.add_object(goal)
    scene.add_object(floorplane)
    scene.add_object(path)

def mouseHandler(scene, evt, msg):
    if evt.type =="mousedown": #check if click
        talker()
        goal.update_attributes(position=(robot.targetPos[0], 0, robot.targetPos[2]), color=(0,255,0)) #update goal position + color
        scene.update_object(goal)

scene.update_object(floorplane, clickable=True, evt_handler=mouseHandler)
scene.run_tasks()