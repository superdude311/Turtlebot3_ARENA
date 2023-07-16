import numpy as np
from arena import *
import re

scene = Scene(host="arenaxr.org", scene="maptest", namespace="matthewkibarian")

def read_pgm(filename, byteorder='>'):

    with open(filename, 'rb') as f:
        buffer_ = f.read()
    try:
        header, width, height, maxval = re.search(
            b"(^P5\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n]\s)*)", buffer_).groups()
    except AttributeError:
        raise ValueError("Not a raw PGM file: '%s'" % filename)
    return np.frombuffer(buffer_, dtype='u1' if int(maxval) < 256 else byteorder+'u2', count=int(width)*int(height), offset=len(header)).reshape((int(height), int(width)))
    
A = read_pgm("C:\\Users\\matth\\OneDrive\\Desktop\\map.pgm")
za = (A == 0).sum()
print(za)

@scene.run_once
def main():
    for i in range(A.shape[1]):
        for j in range(A.shape[0]):
            if A[j,i] == 0:
                print(j, i)
                x = A.shape[0]-j-(A.shape[0]/2)
                z = i-(A.shape[1]/2)
                sf = 4 #scale factor for pos/scale
                scene.add_object(Box(object_id=f"box{j}{i}", position=(x/sf,3/2,z/sf), color=(0,0,0), scale=(1/sf, 3, 1/sf), persist=True))
scene.run_tasks()