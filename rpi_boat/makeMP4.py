#!/usr/bin/python

import cv2
import os
import sys
import glob
import numpy as np

"""
print(sys.argv[1])
print(sys.argv[1].split('.')[0])
print(sys.argv[1].split('.')[0].split('_'))
print(sys.argv[1].split('.')[0].split('_')[3])
print(sys.argv[2])
print(len(sys.argv))

print(sys.argv[1])
print(sys.argv[1].split('.')[0].split('_')[3])
print(os.path.dirname(os.path.abspath(sys.argv[1])))
"""



fl = list()
for arg in sys.argv[1:]:
    fl.append([arg.split('.')[0].split('_')[5],arg])
fl.sort(key=lambda a:a[0])

vw = cv2.VideoWriter(os.path.join(os.path.dirname(os.path.abspath(sys.argv[1])),'output.mp4'),cv2.VideoWriter_fourcc('M','J','P','G'),5,(2560,720))
for f in fl:
    print(f[1])
    img = cv2.imread(f[1])
    vw.write(img)
vw.release()

