import os
import time

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
    mav_mgr = MavlinkManager()
    mav_connection = mav_mgr.mav_connection

    ### Connection When there is no MavProxy and you are connecting to the SITL directly (no Mission Planner)
    # mav_connection = mavutil.mavlink_connection('tcp:127.0.0.1:5760')

    # Create a Flight Control Commander Object
    fc = FltCtlCommander(mav_connection)

    # Wait for the first heartbeat and print it out to confirm you are connected
    mav_connection.wait_heartbeat()
    print("Heartbeat from : " + str(mav_connection.target_system) + " / " + str(mav_connection.target_component))

    # Start your command sequence
    mode_id = mav_connection.mode_mapping()['GUIDED']
    if fc.send_mode_change_to_flt_ctlr(mode_id):
        if fc.send_arm_disarm_command(1, override=True):
            print(fc.send_takeoff_command())
  
    time.sleep(10)

    # Start a preplanned mission
    # # fc.send_start_mission_command()

    # Send a target to a waypoint in guided mode
    target_string_from_non_python = "{\"lat\": 39.0179776, \"lon\": -104.8936, \"alt\": 2170.0}"
    target_string_from_python = '{"lat": 39.0179776, "lon": -104.8936, "alt": 2170.0}'
    target_dictionary = {"lat": 39.0179776, "lon": -104.8936, "alt": 2170.0}
    
    # pick which form of target you want
    target = target_dictionary

    fc.send_guided_target_to_flt_ctlr(target)

    # Run a continuous loop to listen and place commands
    while 1:
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
            print(msg)


# Initialize the code
if __name__ == "__main__":
    main()
