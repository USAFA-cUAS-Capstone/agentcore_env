import json

# from classes.Waypoint import Waypoint

class Route:

    ''' Collection of Waypoints to makeup a Route '''

    def __init__(self):

        self.route = {}
        self.data = {}

    def __iter__(self):
        for key in self.route:
            yield (key, self.route[key].__dict__)

    def appendWaypoint(self, waypoint):
        
        count = len(self.route)
        self.route[str(count).zfill(3)] = waypoint