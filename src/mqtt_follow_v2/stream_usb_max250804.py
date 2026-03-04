# Copyright (C) Meridian Innovation Ltd. Hong Kong, 2020. All rights reserved.
#
import sys
sys.path.append("/home/test/myenv/lib/python3.11/site-packages")
import os
import signal
import time
import logging
import serial
import numpy as np

try:
    import cv2 as cv
except:
    print("Please install OpenCV (or link existing installation)"
          " to see the thermal image")
    exit(1)

from senxor.mi48 import MI48, format_header, format_framestats
from senxor.utils import data_to_frame, remap, cv_filter,\
                         cv_render, RollingAverageFilter,\
                         connect_senxor

def tempperature_out():

    global mi48
    mi48, connected_port, port_names = connect_senxor()

    # set desired FPS
    if len(sys.argv) == 2:
        STREAM_FPS = int(sys.argv[1])
    else:
        STREAM_FPS = 15
    mi48.set_fps(STREAM_FPS)

    # see if filtering is available in MI48 and set it up
    mi48.disable_filter(f1=True, f2=True, f3=True)
    mi48.set_filter_1(85)
    mi48.enable_filter(f1=True, f2=False, f3=False, f3_ks_5=False)
    mi48.set_offset_corr(0.0)

    mi48.set_sens_factor(100)
    mi48.get_sens_factor()
    dminav = RollingAverageFilter(N=10)
    dmaxav = RollingAverageFilter(N=10)
    # initiate continuous frame acquisition
    with_header = True
    mi48.start(stream=True, with_header=with_header)

    data, header = mi48.read()
    ###print(data.max())
    max_out=data.max()
    #####mi48.stop()
    min_temp = dminav(data.min())  # + 1.5
    max_temp = dmaxav(data.max())  # - 1.5
    frame = data_to_frame(data, (80,62), hflip=False);
    frame = np.clip(frame, min_temp, max_temp)

    return frame,max_out

out=tempperature_out()
print(out[0])
#print(out[0])
#print(out[1])
#import csv
#with open('data.txt', 'w', newline='') as file:
#    writer = csv.writer(file)
#    writer.writerows(out[0])