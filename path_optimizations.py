import math

import numpy as np
from CGALPY.Ker import Segment_2, Circle_2, Direction_2

from Ker import *
from discopygal.bindings import Segment_2, Point_2

from rdp import rdp

def douglas_poiker(points):
    """returns optimaized list of Point_2 according to douglas poiker algorithm"""
    #TODO
    pass

def parse_path(rays: [Ray_2], last_point: Point_2):
    """
    returns list of tuples: (starting angle, distance)
                     (p2)
                    /    \
    prev_distance  /      \ curr_distance
                  /        \
               (p1)        (p3)
    """
    angle_distance_list = []
    p0 = np.zeros(2)
    p1 = np.zeros(2)
    p2 = np.zeros(2)
    print(f"Calculated {len(rays)} rays")
    for i in range(1, len(rays)-1):
        p0[0] = rays[i-1].source().x().to_double()
        p0[1]= rays[i-1].source().y().to_double()
        p1[0] = rays[i].source().x().to_double()
        p1[1]= rays[i].source().y().to_double()
        p2[0] = rays[i + 1].source().x().to_double()
        p2[1] = rays[i + 1].source().y().to_double()
        #if p2[0]
        prev_distance = np.linalg.norm(p1-p0)
        curr_distance = np.linalg.norm(p2-p1)
        #time_for_seg = time_per_length(distance)
        angle1 = get_rad_from_direction(rays[i].direction())
        angle0 = get_rad_from_direction(rays[i - 1].direction())
        angle = min(angle1 - angle0, np.pi - (angle1 - angle0))
        if i == 1:#first iteration
            print(f'First Angle: {math.degrees(angle0)}, Distance: {prev_distance}')
            angle_distance_list.append((math.degrees(angle0),prev_distance))
        print(f'Angle: {math.degrees(angle)}, Distance: {curr_distance}')

        angle_distance_list.append((math.degrees(angle), curr_distance))
        if i==len(rays)-2:
            last_p = np.zeros(2)
            last_p[0] = last_point.x().to_double()
            last_p[1] = last_point.y().to_double()
            distance = np.linalg.norm(last_p-p2)
            angle2 = get_rad_from_direction(rays[i+1].direction())
            angle = min(angle2 - angle1, np.pi - (angle2 - angle1))
            print(f'Last Angle: {math.degrees(angle)}, Distance: {distance}')
            angle_distance_list.append((math.degrees(angle), distance))
    return angle_distance_list

class PathSection:
    def __init__(self, ker_element):
        """contains all the info that the robot needs to move that section"""
        self.KerElement = None
        self.isCircle = False

        self.isGlide = False #if true, means speed is changing in this segment
        self.intervals = 20

        self.time = 0
        self.distance = 0
        self.arc_angle = 0
        self.raduis = 0

        self.speed_start = 0 #both speeds should be the same if this is not glide motion
        self.speed_end = 0

        self.acceleration = 0

        self.angle_start = 0 #angle at start of movement. for straight segment this is simply its angle
        self.angle_end = 0 #angle at the end of movement (should be the same as start for segment)

        self.should_stop = True #timeout == sleep time ---> full stop at the end of movement.
                                 #if sleep time is less than timeout, movement will continue (without force)
        self.is_first_movement = False
        self.is_last_movement = False

def get_rad_from_direction(direction: Direction_2):
    return math.atan2(direction.dy().to_double(), direction.dx().to_double())

def parse_path(path):
    """ assume path is list of segments and circles that connect each segment to the other
        speed/acceleration should be determined afterwards!
        """
    path_for_robot=[]
    for i, ker_item in enumerate(path):
        section = PathSection(ker_item)
        if i == 0:
            section.is_first_movement = True
        if i == len(path)-1:
            section.is_last_movement = True

        if type(ker_item) is Segment_2:
            section.isCircle = False
            section.distance = math.sqrt(ker_item.squared_length().to_double())
            section.angle_start = math.degrees(get_rad_from_direction(ker_item.direction()))

        elif type(ker_item) is Circle_2:#its a circle
            section.isCircle = True
            section.raduis = math.sqrt(ker_item.squared_radius().to_double())
            section.arc_angle = 0#to complete. not sure how to get it



        path_for_robot.append(section)

    return path_for_robot