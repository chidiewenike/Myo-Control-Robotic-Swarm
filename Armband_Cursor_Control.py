'''
Authors: Chidi Ewenike, Aamodh Suresh, Tomas Torres, Ramon Duran
'''

#! /usr/bin/python

import matplotlib.pyplot as plt 
import numpy as np
import math
from math import pow,atan2,sqrt,cos,sin,asin,pi
import rosbag
import rospy
import pyautogui
from std_msgs.msg import UInt8, String
from geometry_msgs.msg import Twist, Vector3
from ros_myo.msg import MyoArm
from ros_myo.msg import EmgArray
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Imu
import pyautogui, sys
import time
import tf
import threading

#custom class for Kalman Filter
from KF_class.kalman import KF
from KF_class.myo_system_1 import A,B,C,F
from KF_class.kalman_settings_1 import R,Q,K

first = True
second = True
first_run = True
but_right = False
but_left = False
angArr=[0.0,0.0,0.0]
magnetometer=[0.0,0.0,0.0,0.0]
zt = [0.0,0.0,0.0,0.0,0.0,0.0]
gyroscope=[0.0,0.0,0.0]
gesture = 0

confirmation = 1
theta = 0
x_pix_val = 0
y_pix_val = 0
x_theta_act = 0
y_theta_act = 0
gest_count = 0
xp = 0
yp = 0
z = 0
curr_gest = 0
time_val = 0
start = 0
end = 0
x_prime = [0]
y_prime = [0]
z_prime = [0]
x_ref = [0]
y_ref = [0]
z_ref = [0]
angArr_ref = [0.0,0.0,0.0]
conf_val = 0
conf_start = 0
conf_end = 0
conf_run = True
is_zero = 0
k_x = 0.0
k_y = 0.0
pixel_x_max = 0
pixel_y_max = 0

# callback function for the IMU subscriber node
def callback(data):
    global magnetometer
    global gyroscope 
    global angArr
    global second
    global x_prime
    global y_prime
    global z_prime 
    global x_ref
    global y_ref
    global z_ref
    global angArr_ref

	# assigns the subscribed orientation values to the magnetometer and the gyroscope array
    magnetometer = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
    gyroscope = np.array([data.angular_velocity.x,data.angular_velocity.y,data.angular_velocity.z])

    # converts quartonions to ueler angles(radians)
    angArr = np.array(tf.transformations.euler_from_quaternion(magnetometer))

    # assigns the first measurement to variables in order to be able to get reference point
    if second:
        x_prime, y_prime, z_prime = angArr

    # conditions that will translate the reference point to 0 for all axis 
    x_ref = angArr[0] - x_prime
   
    y_ref = angArr[1] - y_prime
    
    z_ref = angArr[2] - z_prime
    
    # new referenced array
    angArr_ref = np.array([x_ref,y_ref,z_ref])

    # sets the second condition to false so only the first measurement is collected
    second = False

# controls incoming gestures
def gest_cont():
    global but_left
    global but_right

    # if wave-left...
    if gesture == 2:

        # left-click down is activated
        pyautogui.mouseDown(button='left')
        but_left = True

    # if wave-right...    
    elif gesture == 3:

        # right-click down is activated
        pyautogui.mouseDown(button='right')
        but_right = True

    # if rested and wave-left or wave right previously happened...    
    elif gesture == 0 and (but_right or but_left):
        if but_right:

            # right-button up is activated
            pyautogui.mouseUp(button='right')
            but_right = False
        elif but_left:

            # left-button up is activated
            pyautogui.mouseUp(button='left')
            but_left = False

# timer to filter continuous gestures            
def timer_tracker():
    global time_val
    global conf_val
    global end
    global start
    global conf_start
    global conf_end
    global first_run
    global conf_run
    global curr_gest 
    global gesture

    # if the timer is not running...
    if first_run: 

        # starts timer by assigning the current time to start
        start = time.time()

        # deactivates the first run boolean
        first_run = False

        # assigns the detected gesture to curr_gest
        curr_gest = gesture

    # if timer is less than 1.5, the gesture hasn't changed, and the gestures are not wave left or right...    
    elif time_val < 1.5 and gesture == curr_gest and not (gesture == 2 or gesture == 3) :
        
        # the current time is set to end
        end = time.time() 

        # the timer is set by taking difference of current time and initial time
        time_val = (end - start)

    # if timer is greater than 1.5, the gesture hasn't changed, and the gestures are not wave left or right...    
    elif time_val > 1.5 and gesture == curr_gest and not (gesture == 2 or gesture == 3):
        
        # if the gesture is 1 (fist)...
        if curr_gest == 1:

            # scroll up is activated
            pyautogui.scroll(1) #left click

        # if the gesture is 4 (spread)...    
        elif curr_gest == 4: 

            # scroll down is activated
            pyautogui.scroll(-1)
        
        # timer is reset
        time_val = 0
        first_run = True
        
    # if the gesture changes...    
    elif not curr_gest == gesture:

        # timer is reset
        time_val = 0
        first_run = True



# gestcallback function for the gesture subscriber node 
def gestcallback(gest):
    global gesture 

    # vector that contains the gesture data 
    gesture = gest.data 

# processes the necessary data that is already in ueler angles
def process():
    global gest_count
    global x_theta_act
    global y_theta_act
    global z
    gest_count =+ 1

    #assigns the data given by the Kalman Filter to variables so that they could 
    #be converted to pixel values 
    x_theta_act, y_theta_act, z = [theta_x,theta_y,theta_z]

    # runs the conversion function 
    conversion()

# converts the calculated angles to pixels
def conversion():
    global xp
    global yp
    global x_pix_val
    global y_pix_val

	# converts the x-orientation values to pixels and realigns the orig
    # pix_val = theta_act*k where k = pival/max_angle
    x_pix_val = x_theta_act*k_x
    x_pix_val = x_pix_val + pixel_x_max

	# converts the y-orientation values to pixels and realigns the origin
    y_pix_val =  -(k_y)* y_theta_act
    y_pix_val = y_pix_val + pixel_y_max

	# checks for values resulting from noise and arm-shake error. If true, 
	# the angle in radians is converted to pixels to move the cursor 
    if ((x_pix_val >= 0) and (x_pix_val <= 2999)):
		xp = x_pix_val
		#print("[[[[***XP***]]]: ", xp)
    if ((y_pix_val >= 0) and (y_pix_val <= 1999)):
		yp = y_pix_val

def pixel_angle_constant_calculator():
    global k_x
    global k_y
    global pixel_x_max
    global pixel_y_max
    
    pix_x, pix_y = pyautogui.size()
    pixel_x_max = pix_x / 2
    pixel_y_max = pix_y / 2
    k_x = pixel_x_max / 0.61086
    k_y = pixel_y_max / 0.43633

if __name__ == '__main__':
    #-------------------KF set-up ------------------------------
    #create the object from kalman filter class
    filter = KF(A,B,C,R,Q,K)
    mt_1 = angArr[0]      #theta x
    mt_2 = angArr[1]      #theta y 
    mt_3 = angArr[2]      #theta z
    mt_4 = 0.0            #omega x
    mt_5 = 0.0            #omega y 
    mt_6 = 0.0            #omega z

    mt_ = [[mt_1],
           [mt_2],
           [mt_3],
           [mt_4],
           [mt_5],
           [mt_6]]

    #previous covariance
    St_1 = 0.0
    St_2 = 0.0
    St_3 = 0.0
    St_4 = 0.0
    St_5 = 0.0
    St_6 = 0.0

    St_ = [[St_1,0,0,0,0,0],
           [0,St_2,0,0,0,0],
           [0,0,St_3,0,0,0],
           [0,0,0,St_4,0,0],
           [0,0,0,0,St_5,0],
           [0,0,0,0,0,St_6]]

    #initial condition of kalman filter
    last_pkg = [mt_, St_]



    ut = [[1.2*angArr_ref[0]],
          [1.2*angArr_ref[1]],
          [angArr_ref[2]]]
    #----------------End of KF set up --------------------

#--------------------End of Definitions and Set-up--------
    #name of the node 
    rospy.init_node('listener', anonymous=True)

    r= rospy.Rate(1000) # 1000hz

	# the subscriber function for imu and gesture sensors
    rospy.Subscriber("myo_imu", Imu, callback)
    rospy.Subscriber("myo_gest", UInt8, gestcallback)
    # publishes the pixel values 
    pixel_pub = rospy.Publisher("mouse_pixels",Pose,queue_size=1000)

    # condition which asks the user to confirm what they want their reference to be
    while confirmation:
        confirmation = input("Type 0 to confirm your global reference frame:  ")
        pixel_angle_constant_calculator()
    # keeps count of how many times the loop runs
    counter = 0

    # loop that runs until crtl+c is pressed 
    while not rospy.is_shutdown():

        zt = [[angArr_ref[0]],       #theta z angular orientation
              [angArr_ref[1]],       #theta y angular orientation
              [angArr_ref[2]],       #theta y angular orientation
              [gyroscope[0]],       #omega z angular velocity
              [gyroscope[1]],       #omega y angular velocity
              [gyroscope[2]]]       #omega y angular velocity

    #----------- Beginnign of Kalman Filter -----------

        # main function to compute the Kalman Filter
        pkg = filter.KF_compute(last_pkg[0], last_pkg[1], ut, zt)
       
        # separate state vector into individual arrays
        last_pkg = pkg 
        mt = pkg[0]
        #output feedback controlle
        ut = np.dot(F,mt)
        #KF results you can separate the list and plot individually
        #according to the mt index shown below
        theta_x = mt[0]
        theta_y = mt[1]
        theta_z = mt[2]
        omega_x = mt[3]
        omega_y = mt[4]
        omega_z = mt[5] 
    #----------------End of Kalman Filter -------------------
 
        process()
        # published the pixel values to a topc name /mouse_pixels
        pixel_msg = Pose()
        pixel_msg.position.x = xp[0]
        pixel_msg.position.y = yp[0]
        pixel_pub.publish(pixel_msg)
        
        #pyautogui controls the computer mouse
        pyautogui.moveTo(xp,yp)
        counter = counter + 1
        timer_tracker()
        gest_cont()
        #loops at the specific rate of the value specified in r 
        r.sleep()
        rospy.spin
# - - - - - - - - - - END of PROGRAM - - - - - - - - - - - - 
