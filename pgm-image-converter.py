import numpy as np

import cv2
import re

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
    return np.frombuffer(buffer_,
                         dtype='u1' if int(maxval) < 256 else byteorder+'u2',
                         count=int(width)*int(height),
                         offset=len(header)
                         ).reshape((int(height), int(width)))


    
A = read_pgm("C:\\Users\\matth\\OneDrive\\Desktop\\map.pgm")
za = (A == 0).sum()
print(za)
'''
A = np.array(np.dstack((A,A,A)))
print(A[:,:,0])
A2 = cv2.imread("C:\\Users\\matth\\OneDrive\\Desktop\\map.png")
print(A2[:,:,0])
cv2.imwrite('map1.png', A)
'''