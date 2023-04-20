import os
import time
import random
import socket, sys, queue, cv2, datetime, pytz, imutils, struct
from threading import Thread, event
import numpy as np

os.environ['MAVLINK20'] = '1'
from pymavlink import mavutil
# from MAVProxy_v2 import Controller
from Controller import Controller
import serial.tools.list_ports

from fltctl_commander_class import FltCtlCommander

HOST='';
PORT=4555;

# bufferless VideoCapture
class VideoCapture:

  def __init__(self, port): # port is the index of the USB camera connection, the # in "vid#" on the pi
    self.cap = cv2.VideoCapture(port)
    self.q = queue.Queue()
    t = Thread(target=self._reader)
    t.daemon = True
    t.start()

  # read frames as soon as they are available, keeping only most recent one
  def _reader(self):
    while True:
      ret, frame = self.cap.read()
      if not ret:
        break
      if not self.q.empty():
        try:
          self.q.get_nowait()   # discard previous (unprocessed) frame
        except queue.Empty:
          pass
      self.q.put(frame)

  def read(self):
    return self.q.get()

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

    # create image socket
    cam = VideoCapture(2)

    ### Connection When there is no MavProxy and you are connecting to the SITL directly (no Mission Planner)
    # mav_connection = mavutil.mavlink_connection('tcp:127.0.0.1:5760')

    # Create a Flight Control Commander Object
    fc = FltCtlCommander(mav_connection)

    # Wait for the first heartbeat and print it out to confirm you are connected
    mav_connection.wait_heartbeat()
    print("Heartbeat from : " + str(mav_connection.target_system) + " / " + str(mav_connection.target_component))

    mav_connection.mav.command_long_send(mav_connection.target_system, mav_connection.target_component, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 
                                         0,33,500000,0,0,0,0,0) #33 is GPS message, 500,000 is microseconds (.5 seconds)
    ackmsg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True)
    print(ackmsg)
    #print(ack_msg[9])
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
    target_dictionary = {"lat": 39.0183, "lon": -104.893, "alt": 2170.0}
    
    # TEST 1
    location1 = {"lat": 39.0183376, "lon": -104.8935127, "alt": 2171.0}
    location2 = {"lat": 39.0184491, "lon": -104.8937944, "alt": 2172.0}
    location3 = {"lat": 39.0185648, "lon": -104.8932686, "alt": 2173.0}
    #location4 = {"lat": 39.018, "lon": -104.895, "alt": 2170.0}
    #location5 = {"lat": 39.0181, "lon": -104.894, "alt": 2170.0}
    #location6 = {"lat": 39.0182, "lon": -104.894, "alt": 2170.0}
    location_bank = [location1, location2, location3] #, location4, location5, location6]

    count = 0
    # Run a continuous loop to listen and place commands

    
    # pick which form of target you want
    
    #TEST 2

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
        direction = 1
        if yaw < 0:
            yaw = np.abs(yaw)
            direction = -1
        '''
        # TEST 1
        if count % 50 == 0:
            fc.send_guided_target_to_flt_ctlr(location_bank[random.randint(0, 2)])
            

        # TEST 2
        '''
        if (count == 50):
            [phi, rho] = Controller(cam.read())
            #del_yaw = np.rad2deg(phi)
            del_yaw = phi
            new_heading = (msg.hdg + (del_yaw*100))/100
            test_heading1 = 45
            test_heading2 = -90
            test_heading3 = 350
            if (new_heading < 0):
                new_heading += 360
            
            if (new_heading > 360 or new_heading < -360):
                new_heading = (new_heading % 360) * 360
            
           #print(new_heading)
        
        if count > 200:
            count = 0

        elif count > 150:
            
            msg = mav_connection.mav.command_long_send(
                mav_connection.target_system,
                mav_connection.target_component,
                mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                0,
                test_heading1,
                15,
                0, # number determines CW (1), CCW (-1)
                0, # 0 is absolute heading, 1 is relative
                0, 0, 0)
            ack_msg = mav_connection.recv_match(type = 'COMMAND_ACK', blocking = True)
            print(ack_msg)
            
        
        elif count > 100:
            
            msg = mav_connection.mav.command_long_send(
                mav_connection.target_system,
                mav_connection.target_component,
                mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                0,
                test_heading2,
                15,
                0, # number determines CW (1), CCW (-1)
                0, # 0 is absolute heading, 1 is relative
                0, 0, 0)
            ack_msg = mav_connection.recv_match(type = 'COMMAND_ACK', blocking = True)
            print(ack_msg)
        
        elif count > 50:
            
            msg = mav_connection.mav.command_long_send(
                mav_connection.target_system,
                mav_connection.target_component,
                mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                0,
                test_heading3,
                15,
                0, # number determines CW (1), CCW (-1)
                0, # 0 is absolute heading, 1 is relative
                0, 0, 0)
            ack_msg = mav_connection.recv_match(type = 'COMMAND_ACK', blocking = True)
            print(ack_msg)
            
            '''
        
        
            # message SET_POSITION_TARGET_LOCAL_NED 0 0 0 9 2503 0 0 0 0 0 0 0 0 0 0 yaw 0 # may need to set TARGET to OFFSET, 


        
        

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
        #    print(msg.hdg)
            current_heading = int(msg.hdg)
            


# Initialize the code
if __name__ == "__main__":
    main()
