import os
import time
import numpy as np
import cv2 as cv
from timeit import default_timer as timer

os.environ['MAVLINK20'] = '1'
from pymavlink import mavutil
import serial.tools.list_ports

from fltctl_commander_class import FltCtlCommander

class MavlinkManager:

    def __init__(self, verbose=False):

        self._verbose = verbose
        self.port_connected = False
        self.mav_connection = None

        self._initialize_the_port()

    def _initialize_the_port(self):
        ''' Gets the port the Cube is connected to '''

        ports = list(serial.tools.list_ports.comports())
        for p in ports:
            print(p)
            if any(flt_ctlr in p.description for flt_ctlr in ["CubeBlack", "Cube", "CUBE", "USB Serial Device"]) and self.port_connected is False:
                print("#################### Establishing Connection to Cube ##########################")
                _serial_port = serial.Serial(p.device)
                _serial_port.close()    # Need to close for window's devices / will deny access when trying to set mav_connection
                self.port_connected = True
                # if self._verbose:
                #     print("Serial Port = "+ str(_serial_port))
                print("Serial Port = "+ str(_serial_port))
                self.serial_port = _serial_port
                # setup the mavconnection to that port and ask for a heartbeat
                if self.mav_connection:
                    self.mav_connection = None
                self.mav_connection = mavutil.mavlink_connection(self.serial_port.port, baud=9600)

        if not self.port_connected:
            self.mav_connection = mavutil.mavlink_connection('udpin:127.0.0.1:14551')
            # self.mav_connection = mavutil.mavlink_connection('tcp:127.0.0.1:5760')
            #*******self.mav_connection.target_system = self.sysid

def main():

    # Start connection listening to a TCP (direct to SITL) or UDP (MAVProxy) port
    ### Connection when the MavProxy is on another computer (The IP and port may vary)
    # mav_connection = mavutil.mavlink_connection('udpin:192.168.1.76:14551')

    ### Connection when the MavProxy is on this computer
    # mav_connection = mavutil.mavlink_connection('udpin:127.0.0.1:14551')

    ### Connection when the Cube is connected to the computer
    mav_mgr = MavlinkManager()
    mav_connection = mav_mgr.mav_connection

    ### Connection When there is no MavProxy and you are connecting to the SITL directly (no Mission Planner)
    # mav_connection = mavutil.mavlink_connection('tcp:127.0.0.1:5760')

    # Create a Flight Control Commander Object
    fc = FltCtlCommander(mav_connection)

    # Wait for the first heartbeat and print it out to confirm you are connected
    mav_connection.wait_heartbeat()
    print("Heartbeat from : " + str(mav_connection.target_system) + " / " + str(mav_connection.target_component))

    mav_connection.mav.commnad_long_send(mav_connection.target_system, mav_connection.target_component, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 
                                         0,33,500000,0,0,0,0,0) #33 is GPS message, 500,000 is microseconds (.5 seconds)
    ackmsg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True)
    print(ackmsg)

    # Start your command sequence
    mode_id = mav_connection.mode_mapping()['GUIDED']
    if fc.send_mode_change_to_flt_ctlr(mode_id):
        #pass
        if fc.send_arm_disarm_command(1, override=True):
            print(fc.send_takeoff_command())

    time.sleep(10)

    # Start a preplanned mission
    # # fc.send_start_mission_command()

    # Send a target to a waypoint in guided mode
    target_string_from_non_python = "{\"lat\": 39.0179776, \"lon\": -104.8936, \"alt\": 2170.0}"
    target_string_from_python = '{"lat": 39.0179776, "lon": -104.8936, "alt": 2170.0}'
    target_dictionary = {"lat": 39.0188, "lon": -104.8931909, "alt": 2170.0}
    
    # pick which form of target you want
    target = target_dictionary

    fc.send_guided_target_to_flt_ctlr(target)

    # Run a continuous loop to listen and place commands
    
    count = 0

    while 1:
        '''
        ***IMPORTANT***
        Receives GPS at a rate of 2 Hz.
        Receive info from Shabri's radar sim. 
        Update target every 2 cycles (for now).
        '''

        

        count += 1

        '''
        if count % 25  == 0:
            # Receive new target
            newtarget = {"lat": 39.0188, "lon": -104.89313, "alt": 2170.0}
            if count == 50:
                newtarget = {"lat": 39.0186999, "lon": -104.893, "alt": 2170.0}
            if count == 75:
                newtarget = {"lat": 39.0186999, "lon": -104.8931909, "alt": 2170.0}
            target = newtarget
            fc.send_guided_target_to_flt_ctlr(target)
        '''
        if count >= 23:
            #call controller
            [phi, rho, z] = Controller()
            yaw = np.deg2rad(phi)
            


        

        # Look at specific messages from the flight controller (Cube)
        # List of possible messages are found here: https://ardupilot.org/dev/docs/mavlink-requesting-data.html
        msg = mav_connection.recv_match(
            type=['GLOBAL_POSITION_INT', 'POSITION_TARGET_GLOBAL_INT'], blocking=True)

        # Look at all messages from the flight controller (Cube)
        # msg = mav_connection.recv_match(blocking=True, timeout=1.0)

        # Filter which message you want to see and operate on.  Realize messages can come in at different rates and they
        # come in one at a time (i.e. once per while loop cycle) so you can't simultaneously read two messages per cycle
        msg_type = msg.get_type()
        if msg_type == 'GLOBAL_POSITION_INT':
        # if msg_type == 'POSITION_TARGET_GLOBAL_INT':
            print(msg)



  

def Controller():
    threshold = 100
    i = 0
    # frameNumber = 0
    # avgAdder = 0

    while(i < 5):
        cam = cv.VideoCapture(i)
        if(cam.isOpened() == True):
            break
        i = i+1
        

    # start = timer()
    ret, img = cam.read()
    invert = cv.bitwise_not(img)
    edgeHeight = img.shape[0]
    edgeWidth = img.shape[1]

    gray_image = cv.cvtColor(img, cv.COLOR_BGR2GRAY)    #Grayscale
    blurred = cv.GaussianBlur(gray_image, (7, 7), 0)    #Blur Images
    # blurred = cv.Canny(gray_image, 50,200)
    ret,thresh = cv.threshold(blurred, threshold,255,cv.ADAPTIVE_THRESH_MEAN_C)    #Convert to Binary
    contours,hierarchy = cv.findContours(thresh, 1, 2) #Find Contours
    cnts = contours[0]

    if(len(contours) > 0):
        j = 0
        sorted_contours=sorted(contours, key=cv.contourArea, reverse= True)
        
        while(j < len(contours)):
            largest_item= sorted_contours[j]
            (bigx, bigy, bigw, bigh) = cv.boundingRect(largest_item)
            j = j+1
            
            if not(bigx == 0 or bigy == 0 or bigw+bigx == edgeWidth or bigh+bigy == edgeHeight):
                break
                
            else:
                banana = 1

    cv.drawContours(img, largest_item, -1, (255,0,0),10)
    # cv.drawContours(img, cnts, -1, (0, 255, 0), 2)
    (x,y),radius = cv.minEnclosingCircle(largest_item)
    center = (int(x),int(y))
    radius = int(radius)
    cv.circle(img,center,radius,(0,0,255),2)

    M = cv.moments(thresh)

    # # calculate x,y coordinate of center
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])


    #this might get checked earlier to get rid of edge contours
    height = int(img.shape[0])
    heightfor = int(height/2)
    width = int(img.shape[1]/2)

    #Determine Yaw
    Yaw = 0
    Yaw = cX - width
    Yaw = Yaw*.0359

    #Determine Z
    Alt = 0
    Alt = cY - (height/2 + height*.1)
    Alt = Alt * 1 #Change the constant to fine tune altitude control

    #print(cv.contourArea(contours))
    cv.line(img, (width,heightfor), center, (255,255,0), 4)
    cv.putText(img, "centroid", (cX - 25, cY - 25),cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    cv.putText(img, str(Yaw), (height-10, width-50),cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    cv.putText(img, str(Alt), (height-10, width-25),cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    # cv.imshow('ret', thresh)

    cv.imshow('this', thresh)
    cv.imshow('me', img)

    cam.release()

    cv.destroyAllWindows()

    return_array = [Yaw, Alt]

    return return_array

# Initialize the code
if __name__ == "__main__":
    main()
