import json

class Waypoint:

    ''' This class is used to structure Waypoint messages into 
        and from Mavlink messages '''

    # MISSION_ITEM
    # command   uint16_t
    # param1    float   (pitch - unused for copter)
    # param2    float   (unused)
    # param3    float   (unused)
    # param4    float   (unused)
    # x         int32_t (unused)
    # y         int32_t (unused)
    # z         float   (unused)

    def __init__(self, lat=0.0, lon=0.0, alt=0.0, speed_at_wp=0.0, action_at_wp=""):

        self.lat = lat
        self.lon = lon
        self.alt = alt
        self.speed_at_wp = speed_at_wp
        self.action_at_wp = action_at_wp

    def read_mavlink_waypoint_into_waypoint_class(self, waypoint_msg):

        # print(str(waypoint_msg.seq) + ": " +str(waypoint_msg.x) + " / " + str(waypoint_msg.y) + " / " + str(waypoint_msg.z))
 
        self.lat = waypoint_msg.x
        self.lon = waypoint_msg.y
        self.alt = waypoint_msg.z

        return self
        
        # return json.dumps(self.__dict__)
