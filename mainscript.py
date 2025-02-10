from inference_sdk import InferenceHTTPClient 
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import math
from pymavlink import mavutil
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

print("Connecting to vehicle...")
port = 14550
vehicle = connect('127.0.0.1:14550', wait_ready=True)  
print(f"Connected to vehicle @ {port}")
class drone_init:
    def __init__(self,x,y,z,):
        self.x = x
        self.y = y
        self.z= z
    def takeoff(self, target_altitude):   #target altitude = half of z.   
        print("Taking off...")
        while not vehicle.is_armable:
            print(" Waiting for vehicle to become armable...")
            time.sleep(1)
        vehicle.mode = VehicleMode("GUIDED")
        vehicle.armed = True
        while not vehicle.armed:
            print(" Waiting for arming...")
            time.sleep(1)
        print("Taking off!")
        vehicle.simple_takeoff(target_altitude)
        while True:
            print(f" Altitude: {vehicle.location.global_relative_frame.alt:.2f}")
            if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:  
                print("Reached target altitude")
                break
            time.sleep(1)
    global object_detect
    def object_detect(self):
        #for now when I enter true it returns true n later I will add camera stream and 
        if (input("Enter state: ") == "true"):
            return True
        else:
            return False
    def scan_area(self):
        print(f"Beginning Scanning Area of Size {self.x}, {self.y}, {self.z} ")
        vehicle.airspeed = 5 
        ## code for drone scanning through the area defined.
        ## drone moves a little and runs object_detect function to search
        state = True
        while state:
            #code for drone moving a bit through area
            if (object_detect()):   
                state = False

    def signal():
        #code to start circling
        print("Beginning to Signal")

try:
    length = 100
    width = 100
    height = 100
    drone = drone_init(length, width, height)
    drone.takeoff((height/2))
    drone.scan_area()
    drone.signal()
finally:
    print("Closing vehicle connection...")
    vehicle.close()