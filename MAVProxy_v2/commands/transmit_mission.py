from cmath import nan
import json
import os
from typing import Dict
from pymavlink import mavutil, mavwp
from agentutils import AgentUtils

from classes.Waypoint import Waypoint
from classes.Route import Route


def write_mission_waypoints_to_agent(msg, mav_connection):
    wp = mavwp.MAVWPLoader()

    mission_dict = json.loads(json.loads(msg))
    print(type(mission_dict))

    frame = mavutil.mavlink.MAV_FRAME_GLOBAL
    takeoffFrame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
    autocontinue = 1
    seq = 0
    p = mavutil.mavlink.MAVLink_mission_item_message(
        mav_connection.target_system,
        mav_connection.target_component,
        seq,
        frame,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        0,
        autocontinue,
        0,
        0,
        0,
        0,
        0,
        0,
        10,
    )
    wp.add(p)
    seq = seq + 1
    p = mavutil.mavlink.MAVLink_mission_item_message(
        mav_connection.target_system,
        mav_connection.target_component,
        seq,
        takeoffFrame,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        autocontinue,
        0,
        0,
        0,
        0,
        0,
        0,
        10,
    )
    wp.add(p)

    for waypoint_id in mission_dict:
        waypoint_data = mission_dict[waypoint_id]
        lat, lon = waypoint_data["lat"], waypoint_data["lon"]
        altitude = waypoint_data["alt"]
        seq = seq + 1
        current = 0
        if seq == len(mission_dict.keys()) + 1:
            # pass
            p = mavutil.mavlink.MAVLink_mission_item_message(
                mav_connection.target_system,
                mav_connection.target_component,
                seq,
                frame,
                mavutil.mavlink.MAV_CMD_NAV_LAND,
                0,
                0,
                5,
                0,
                0,
                0,
                lat,
                lon,
                altitude,
            )
            wp.add(p)
        else:
            p = mavutil.mavlink.MAVLink_mission_item_message(
                mav_connection.target_system,
                mav_connection.target_component,
                seq,
                frame,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0,
                autocontinue,
                0,
                0,
                0,
                0,
                lat,
                lon,
                altitude,
            )
            wp.add(p)

    mav_connection.waypoint_clear_all_send()
    mav_connection.waypoint_count_send(wp.count())

    for i in range(wp.count()):
        print(i)
        print(wp.wp(i))
        mav_connection.mav.send(wp.wp(i))


def read_mission_waypoints_from_agent(mav_connection):
    config = AgentUtils.get_config_dict()
    mav_connection.waypoint_request_list_send()
    waypoint_count = 0

    msg = mav_connection.recv_match(type=["MISSION_COUNT"], blocking=True)
    waypoint_count = msg.count
    # print(msg.count)

    command_reply = {"command_reply": "READ_ROUTE"}
    route_parameters = {}
    route = {}
    for i in range(waypoint_count):
        mav_connection.waypoint_request_send(i)
        msg = mav_connection.recv_match(type=["MISSION_ITEM"], blocking=True)
        route_parameters[str(i).zfill(3)] = {
            "lat": msg.x,
            "lon": msg.y,
            "alt": msg.z,
            "speed_at_wp": 0,
            "action_at_wp": "",
        }
    route["route"] = route_parameters
    command_reply["agent_id"] = config["AGENT_ID"]
    command_reply["reply_parameters"] = route

    mav_connection.mav.mission_ack_send(
        mav_connection.target_system, mav_connection.target_component, 0
    )  # OKAY
    return command_reply


# # Set Home location
# def cmd_set_home(home_location, altitude):
#     mav.mav.command_long_send(
#         mav.target_system, mav.target_component,
#         mavutil.mavlink.MAV_CMD_DO_SET_HOME,
#         1, # set position
#         0, # param1
#         0, # param2
#         0, # param3
#         0, # param4
#         home_location[0], # lat
#         home_location[1], # lon
#         altitude) # alt

# def cmd_get_home():
#     mav.mav.command_long_send(
#         mav.target_system, mav.target_component,
#         mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,
#         0, 0, 0, 0, 0, 0, 0, 0)
#     msg = mav.recv_match(type=['COMMAND_ACK'],blocking=True)
#     print msg
#     msg = mav.recv_match(type=['HOME_POSITION'],blocking=True)
#     return (msg.latitude, msg.longitude, msg.altitude)
