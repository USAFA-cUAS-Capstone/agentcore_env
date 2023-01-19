import os
import time

os.environ['MAVLINK20'] = '1'
from pymavlink import mavutil

from fltctl_commander_class import FltCtlCommander

def main():

    # Start connection listening to a TCP (direct to SITL) or UDP (MAVProxy) port
    ### Connection when the MavProxy is on another computer (The IP and port may vary)
    # mav_connection = mavutil.mavlink_connection('udpin:192.168.1.76:14551')

    ### Connection when the MavProxy is on this computer
    mav_connection = mavutil.mavlink_connection('udpin:127.0.0.1:14551')

    ### Connection When there is no MavProxy and you are connecting to the SITL directly (no Mission Planner)
    print("Trying to connect")
    # mav_connection = mavutil.mavlink_connection('tcp:127.0.0.1:5760')
    print("Connected?")

    # Create a Flight Control Commander Object
    fc = FltCtlCommander(mav_connection)

    # Wait for the first heartbeat and print it out to confirm you are connected
    mav_connection.wait_heartbeat()
    print("Heartbeat from : " + str(mav_connection.target_system) + " / " + str(mav_connection.target_component))

    # time.sleep(5)
    # print("Awake")

    # Start your command sequence
    # mode_id = mav_connection.mode_mapping()['GUIDED']
    if fc.send_mode_change_to_flt_ctlr(4):
    # if fc.send_mode_change_to_flt_ctlr(mode_id):
        if fc.send_arm_disarm_command(1, override=True):
            print(fc.send_takeoff_command())

    time.sleep(10)

    # Start a preplanned mission
    # # fc.send_start_mission_command()

    # Send a target to a waypoint in guided mode
    target_string_from_non_python = "{\"lat\": 39.0179776, \"lon\": -104.8936, \"alt\": 2170.0}"
    target_string_from_python = '{"lat": 39.0179776, "lon": -104.8936, "alt": 2170.0}'
    target_dictionary = {"lat": 39.0179776, "lon": -104.8936, "alt": 2170.0} 

    # Shabri's location stuff goes in here (or in while loop)
    
    # pick which form of target you want
    target = target_dictionary
    #target2 = {"lat": 39.0179776, "lon": -104.894, "alt": 2170.0}
    #target3 = {"lat": 39.0179800, "lon": -104.8936, "alt": 2170.0}
    #target4 = {"lat": 39.0179800, "lon": -104.8931, "alt": 2165.0}

    #targets = [target1, target2, target3, target4]

    # targets.append(target1, target2, target3, target4)

    #for target in targets:
    fc.send_guided_target_to_flt_ctlr(target)
        # Work in progress

    # Run a continuous loop to listen and place commands
    while 1:
        # target = target_dictionary # target = input from shabri
        # fc.send_guided_target_to_flt_ctlr(target) # have send target in loop with refresh rate
        # make refresh rate base on distance 

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
