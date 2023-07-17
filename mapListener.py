#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid
import numpy as np
import cv2
from arena import *
import time

scene = Scene(host="arenaxr.org", scene="livemap1", namespace="matthewkibarian")
startTime = time.time()
box = Box(object_id='box', position=(0,0,0), color=(255,255,255))
lidarMap = None

def callback(data):
    global startTime
    global scene
    global box
    global lidarMap
    lidarMapRaw=np.array(data.data)
    width = data.info.width
    height = data.info.height
    rospy.loginfo(rospy.get_caller_id() + "I heard %s, %s, %s", np.array(data.data), data.info.width, data.info.height)
    lidarMap = lidarMapRaw.reshape((height, width))

@scene.run_once
def box1():
    scene.add_object(box)

@scene.run_forever(interval_ms=2000)
def arenaLoop():
    print(lidarMap)
    if lidarMap is None:
        return
    for i in range(lidarMap.shape[1]):
        for j in range(lidarMap.shape[0]):
            if lidarMap[j,i] >= 75:
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

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/map", OccupancyGrid, callback)
    print('listener running')
    scene.run_tasks()
if __name__ == '__main__':
    listener()