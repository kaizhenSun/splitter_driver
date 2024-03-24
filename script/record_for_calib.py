#!/usr/bin/env python3 
# -*- coding: utf-8 -*-

import ctypes
import os
import shutil
import time
import random
import sys
import threading
import time
import cv2
import numpy as np
# import pycuda.autoinit
# import pycuda.driver as cuda
# import tensorrt as trt
import platform
import rospy
# import gxipy as gx
from PIL import Image as PIL_Image
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

mutex = threading.Lock()
image_id = 11
aps_save_path = "/home/jhang/workspace/splitter_driver/calib_file/calibration_imgs/aps"
event_save_path = "/home/jhang/workspace/splitter_driver/calib_file/calibration_imgs/event"

os.makedirs(aps_save_path, exist_ok=True)
os.makedirs(event_save_path, exist_ok=True)

save_order_aps = False
save_order_event = False
bridge = CvBridge()

class recordThread(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)

    def run(self):
        global image_id
        global save_order_aps
        global save_order_event
        while(True):
            iteration_continue_aps = True
            while(iteration_continue_aps):
                button = input("press c to capture aps, x to exit\n")
                if button == 'c':
                    iteration_continue_aps = False
                if button == 'x':
                    os._exit()
            mutex.acquire()
            save_order_aps = True
            mutex.release()
            iteration_continue_event = True
            while(iteration_continue_event):
                button = input("press c to capture event, x to exit\n")
                if button == 'c':
                    iteration_continue_event = False
                if button == 'x':
                    os._exit()
            mutex.acquire()
            save_order_event = True
            mutex.release()



def aps_callback(data):
    global image_id
    global save_order_aps
    global event_wating_count
    cv_aps_image = bridge.imgmsg_to_cv2(data, "bgr8")
    
    mutex.acquire()
    if save_order_aps:
        event_wating_count = data.header.stamp.secs
        save_path = aps_save_path + "/image" + str(image_id) + ".png"
        cv2.imwrite(save_path, cv_aps_image)
        print("saving aps frame in " + save_path)
        save_order_aps = False
    mutex.release()
    
def event_callback(data):
    global image_id
    global save_order_event
    global save_order_aps
    global event_wating_count
    cv_event_image = bridge.imgmsg_to_cv2(data, "bgr8")
    mutex.acquire()
    if save_order_event:
        save_path = event_save_path + "/image" + str(image_id) + ".png"
        cv2.imwrite(save_path, cv_event_image)
        print("saving event frame in " + save_path)
        save_order_event = False
        image_id = image_id + 1
    mutex.release()


if __name__ == "__main__":
    rospy.init_node("record_node")
    aps_sub = rospy.Subscriber("/image_raw", Image, aps_callback, queue_size=1)
    event_sub = rospy.Subscriber("/visualization_node/image", Image, event_callback, queue_size=1)
    thread1 = recordThread()
    thread1.start()
    rospy.spin()


    
