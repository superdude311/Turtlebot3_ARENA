# groundClick
# purpose: click the ground (a plane) and the robot will drive to that point

from arena import *
import math
scene = Scene(host="arenaxr.org", scene="groundClick", namespace="matthewkibarian")
class Robot:
    def __init__(self):
        self.position = [0,0,0]
        self.rotation = [0,0,0]
        self.model = GLTF(object_id="robot", position=self.position, rotation=self.rotation, scale=(0.25,0.25,0.25), url="store/models/Duck.glb", persist=True)
        self.targetPos = [0,0,0]
        self.xstep = 0 #rule: make sure step is a bit smaller than threshold so it wont get stuck
        self.zstep = 0 #actually with variable step i have to make 2 variable thresholds
        self.xthreshold = 0.05
        self.zthreshold = 0.05
        self.stepMult = 0.01
        self.rstep = 0.01
        self.rthreshold = 10
        self.direction = 1 # would be used as a positive negative multiplier to switch the rotation direction but its borked
    def addModel(self): #add model to ARENA
        scene.add_object(self.model)
    def circleWrap(self, angle):
        if angle < 0: #if negative angle, add 360, e.g. -47 + 360 = 313
            angle += 360
        if angle > 360: #if angle greater than 360, subtract 360, e.g. 388 - 360 = 28
            angle -= 360
        return angle
    def angleDiff(self, angle1, angle2):
        if abs(angle1 - angle2) <= 180:
            return abs(angle1 - angle2)
        if ((angle1-angle2) > 180) or ((angle1-angle2) <= 0): 
            if angle1 - angle2 < 0: #if A1-A2 is negative, e.g. 1-359 = -358 + 360 = 1
                return (angle1 - angle2) + 360
            elif angle1 - angle2 == 0: #if A1-A2 is zero - will this ever happen if the threshold is 10?
                return 0
            elif angle1 - angle2 > 180: #if A1-A2 is greater than 180, e.g. 340-1 = 339, 360-339 = 21
                return 360 - (angle1 - angle2)
    def updateRotation(self): 
        xdist = self.targetPos[0] - self.position[0]
        zdist = self.targetPos[2] - self.position[2]
        if xdist == 0 and zdist == 0:
            return False
        c = math.sqrt(xdist**2 + zdist**2)
        # rrad = math.atan(zdist/xdist)
        rrad = math.asin((abs(xdist)*math.sin(math.pi/2))/c)
        r = (rrad * (180/math.pi))
        r = self.circleWrap(r)
        if zdist > 0 and xdist > 0: #Q4
            r += 270
            r = self.circleWrap(r)
        elif zdist > 0 and xdist < 0:#Q3
            r = 270 - r
            r = self.circleWrap(r)
        elif zdist < 0 and xdist < 0: #Q2
            r += 90
            r = self.circleWrap(r)
        elif zdist < 0 and xdist > 0: #Q1 - takes much longer to rotate to go into Q1 than other quadrants, especially from Q2
            r = 90 - r
            r = self.circleWrap(r)
        if self.angleDiff(r, self.rotation[1]) > self.rthreshold:
            self.rotation[1] += r * self.rstep
        self.rotation[1] = self.circleWrap(self.rotation[1])
        self.model.update_attributes(rotation=self.rotation)
        scene.update_object(self.model)
        return True
    def update(self):
        if self.updateRotation() == False:
            return
        theta = self.rotation[1] * (math.pi/180)
        self.xstep = math.sin(theta + (math.pi/2)) * self.stepMult
        self.zstep = math.cos(theta + (math.pi/2)) * self.stepMult
        if (abs(self.targetPos[0] - self.position[0]) > self.xthreshold or abs(self.targetPos[2] - self.position[2]) > self.zthreshold):
            self.position[0] += self.xstep
            self.position[2] += self.zstep
            self.model.update_attributes(position=self.position)
            scene.update_object(self.model)
        else:
            return
    def updateTargetPos(self, x, z): #updates the target position
        self.targetPos[0] = x
        self.targetPos[2] = z
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
        robot.updateTargetPos(evt.data.position.x, evt.data.position.z) #update target position to mouse click pos
        robot.updateRotation()
        goal.update_attributes(position=(robot.targetPos[0], 0, robot.targetPos[2]), color=(0,255,0)) #update goal position + color
        scene.update_object(goal)
@scene.run_forever(interval_ms=25) #typical interval 100ms
def move():
    robot.update() #update robot position/move rorb
    path.update_attributes(start=Position(robot.position[0],0.01,robot.position[2]), end=Position(robot.targetPos[0], 0.01, robot.targetPos[2]), color=(100,0,255))
    scene.update_object(path) #update line path + color in the loop so the line disappears as the robot drives over it
scene.update_object(floorplane, clickable=True, evt_handler=mouseHandler)
scene.run_tasks()
#this is identifiable