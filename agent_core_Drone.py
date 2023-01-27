import os
import time
import cv2, queue, socket, sys, datetime, pytz, imutils, struct
from threading import Thread, Event
from time import sleep
import numpy as np

os.environ['MAVLINK20'] = '1'
from pymavlink import mavutil
import serial.tools.list_ports

from fltctl_commander_class import FltCtlCommander


# This class initializes a separate thread for sending image frames back to ground station
# using the SndImg function.
# May change to run a Thread object upon init instead of making it a child class
class SndImgC2(Thread):
    def __init__(self, event):
        Thread.__init__(self)
        self.stopped = event

        self.imageProc = 1
        self.imageSave = 0
        self.jpeg_quality = 70  # 0 to 100, higher is better quality, 95 is cv2 default
        self.cap = VideoCapture(2) ##CHECK VIDEO PORT & FIX
        print("Opening Camera Interface : 2")
        self.noNewFile=True
        self.msgFormat='<HHdddddHddddhhh'
        self.MsgHeadr=int("a59f",16) # randomly chosen for experiment(?)
        self.host='127.0.0.1' ##FIX THIS
        self.port=4555 ##FIX THIS

        try:
            self.sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sender.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sender.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        except socket.error:
            print('Failed to create socket')
            sys.exit()

    def run(self):
        while (not self.stopped.is_set()):
            self.SndImg()

    def SndImg(self):

        frame = self.cap.read()
        if(self.imageSave == 1): # save image as file if param set
            cv2.imwrite('DDrone_' + str(int(datetime.datetime.now(tz=pytz.utc).timestamp())) + '_image_' + '.jpg', frame)
        # Do image proc and write to jpg_buffer
        (B, G, R) = cv2.split(frame)
        zeros = np.zeros(frame.shape[:2], dtype = "uint8")
        blueEmphasis = cv2.merge([B, zeros, zeros])
        blueGray = frame[:,:,0]
        blurred = cv2.GaussianBlur(blueGray, (7, 7), 0)
        (T, bThresh) = cv2.threshold(blurred, 50, 255, cv2.THRESH_BINARY_INV)
        (cnts, hierarchy) = cv2.findContours(bThresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        (cnts, hierarchy) = cv2.findContours(bThresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if(len(cnts) > 0):
            biggestcnt = cnts[0]
            (bigx, bigy, bigw, bigh) = cv2.boundingRect(biggestcnt)
            big_area = bigw * bigh
            for (i, c) in enumerate(cnts):
                (newx, newy, neww, newh) = cv2.boundingRect(c)
                box_area = neww * newh
                target_area = cv2.contourArea(cnts[i])
                if target_area > big_area:
                    biggestcnt = cnts[i]

            M = cv2.moments(biggestcnt)
            blueTargets = frame.copy()
            cv2.drawContours(blueTargets, cnts, -1, (0, 255, 0), 2)
            (x,y),radius = cv2.minEnclosingCircle(biggestcnt)
            center = (int(x),int(y))
            radius = int(radius)
            cv2.circle(blueTargets,center,radius,(0,0,255),2)
            if(self.imageSave == 1):
                cv2.imwrite('ProcDDrone_' + str(int(datetime.datetime.now(tz=pytz.utc).timestamp())) + '_image_' + '.jpg', blueTargets)
            blueTargets=imutils.resize(blueTargets,width=320)

        if((len(cnts) > 0) & (self.imageProc == 1)):
                ret_code, jpg_buffer = cv2.imencode(
            ".jpg", blueTargets, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        else:
            ret_code, jpg_buffer = cv2.imencode(
            ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])

        # encode image to send
        dataEncode=np.array(jpg_buffer)
        str_encode = dataEncode.tobytes()

        metaBase=[self.MsgHeadr]
        metaBase.append(np.uint16(84)) # metadata size; THIS SHOULD NOT BE HARDCODED I AM GOING TO FIND LT PATTERSON AND FIGHT HIM
        metaBase.append(datetime.datetime.now(tz=pytz.utc).timestamp())
        #metaBase.append(int(latitude_deg*1e7))
        #metaBase.append(int(longitude_deg*1e7))
        #metaBase.append(int(elevation_m_MSL*10))
        #metaBase.append(int(mav.currentLat_Deg * 1e7))
        #metaBase.append(int(mav.currentLon_Deg * 1e7))
        #metaBase.append(int(mav.currentAlt_mMSL * 10))
        metaBase.append(90) #Azimuth (Deg)
        metaBase.append(0) #Tilt (Deg)
        metaBase.append(0) #Zoom (%)
        metaBase.append(320) #Image Size (Pixels)
        metaBase.append(0) #Blob Present (y or n)
        metaBase.append(0) #Blob Size (Pixels)
        metaBase.append(0) #Blob location on screen (x,y)
        metaBase.append(0) #Object Classification
        metaBase.append(0) #Object Class. certainty
        metaBase.append(0) #Velocity X
        metaBase.append(0) #Velocity Y
        metaBase.append(0) #Velocity Z
        metadata=struct.pack(self.msgFormat,*metaBase)
        #sleep(.05)
        message=metadata+str_encode

        #print(len(message))
        #print(jpg_buffer.shape)
        #print("")
        try:
            self.sender.sendto(message,(self.host,self.port))
        except socket.error as msg:
            print(msg)
            sys.exit()

        #time.sleep(0.2) # allowed time for image processing
        #if chr(cv2.waitKey(1)&255) == 'q':
        #  break

        #except KeyboardInterrupt:
        #    cap.cap.release()
        #    cv2.destroyAllWindows()
        #    break

        self.sender.close

        #cv2.destroyAllWindows()
        #cap.cap.release()
        #sys.exit()

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
            print("looking for connection")
            ### agent_core on same machine as MavProxy(udpin) or direct connect to SITL(tcp)
            self.mav_connection = mavutil.mavlink_connection('udpin:127.0.0.1:14551')
            # self.mav_connection = mavutil.mavlink_connection('tcp:127.0.0.1:5760')

            ### agent_core on different machine as MavProxy(udpin) or direct connect to SITL(tcp)
            # self.mav_connection = mavutil.mavlink_connection('udpin:0.0.0.0:14551')
            # self.mav_connection = mavutil.mavlink_connection("tcp:192.168.1.25:5760")


def main():

    # Start connection listening to a TCP (direct to SITL) or UDP (MAVProxy) port
    ### Connection when the MavProxy is on another computer (The IP and port may vary)
    # mav_connection = mavutil.mavlink_connection('udpin:192.168.1.76:14551')

    ### Connection when the MavProxy is on this computer
    # mav_connection = mavutil.mavlink_connection('udpin:127.0.0.1:14551')

    ### Connection when the Cube is connected to the computer
    #mav_mgr = MavlinkManager()
    #mav_connection = mav_mgr.mav_connection

    ### Connection When there is no MavProxy and you are connecting to the SITL directly (no Mission Planner)
    # mav_connection = mavutil.mavlink_connection('tcp:127.0.0.1:5760')

    # Create a Flight Control Commander Object
    #fc = FltCtlCommander(mav_connection)

    # Wait for the first heartbeat and print it out to confirm you are connected
    #mav_connection.wait_heartbeat()
    #print("Heartbeat from : " + str(mav_connection.target_system) + " / " + str(mav_connection.target_component))

    # Start your command sequence
    #mode_id = mav_connection.mode_mapping()['GUIDED']
    #if fc.send_mode_change_to_flt_ctlr(mode_id):
    #    if fc.send_arm_disarm_command(1, override=True):
    #        print(fc.send_takeoff_command())
  
    #time.sleep(10)

    # Start a preplanned mission
    # # fc.send_start_mission_command()

    # Send a target to a waypoint in guided mode
    #target_string_from_non_python = "{\"lat\": 39.0179776, \"lon\": -104.8936, \"alt\": 2170.0}"
    #target_string_from_python = '{"lat": 39.0179776, "lon": -104.8936, "alt": 2170.0}'
    #target_dictionary = {"lat": 39.0179776, "lon": -104.8936, "alt": 2170.0}
    
    # pick which form of target you want
    #target = target_dictionary

    #fc.send_guided_target_to_flt_ctlr(target)

    # start video thread(s)
    stopFlag = Event()
    SndImgThread = SndImgC2(stopFlag)
    SndImgThread.start()

    # Run a continuous loop to listen and place commands
    while 1:
        # Look at specific messages from the flight controller (Cube)
        # List of possible messages are found here: https://ardupilot.org/dev/docs/mavlink-requesting-data.html
    #    msg = mav_connection.recv_match(
    #        type=['GLOBAL_POSITION_INT', 'POSITION_TARGET_GLOBAL_INT'], blocking=True)

        # Look at all messages from the flight controller (Cube)
        # msg = mav_connection.recv_match(blocking=True, timeout=1.0)

        # Filter which message you want to see and operate on.  Realize messages can come in at different rates and they
        # come in one at a time (i.e. once per while loop cycle) so you can't simultaneously read two messages per cycle
    #    msg_type = msg.get_type()
    #    if msg_type == 'GLOBAL_POSITION_INT':
    #        print(msg)
        time.sleep(1)


# Initialize the code
if __name__ == "__main__":
    main()
