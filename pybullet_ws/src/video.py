# coding=utf-8

import cv2
import argparse
import os
from utils import get_files, pathname

SRC = '/home/mtb/RES4'
width =256
height = 256

FOLDERS = ['EVAL_%d'%d for d in range(1,4)]


for t in FOLDERS:
    i = int(t[5:])
    img_src = pathname(SRC,t)
    video_src = pathname(SRC, 'VIDEO_RES4','output_%d.mp4'%i)

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')# Be sure to use lower case
    out = cv2.VideoWriter(video_src, fourcc, 20.0, (width, height))
    for img in get_files(pathname(img_src,'IMG'), key = lambda f :int(f[4:f.find('.png') ])):
        frame = cv2.imread(img)
        out.write(frame)

    out.release()
    cv2.destroyAllWindows()