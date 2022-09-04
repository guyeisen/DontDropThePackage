import math
from typing import List

import numpy as np

from discopygal.bindings import *

from rdp import rdp


def douglas_poiker(points):
    """returns optimaized list of Point_2 according to douglas poiker algorithm"""
    # TODO
    pass


def parse_path(rays: List[Ker.Ray_2], last_point: Point_2):
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
    for i in range(1, len(rays) - 1):
        p0[0] = rays[i - 1].source().x().to_double()
        p0[1] = rays[i - 1].source().y().to_double()
        p1[0] = rays[i].source().x().to_double()
        p1[1] = rays[i].source().y().to_double()
        p2[0] = rays[i + 1].source().x().to_double()
        p2[1] = rays[i + 1].source().y().to_double()
        # if p2[0]
        prev_distance = np.linalg.norm(p1 - p0)
        curr_distance = np.linalg.norm(p2 - p1)
        # time_for_seg = time_per_length(distance)
        angle1 = get_rad_from_direction(rays[i].direction())
        angle0 = get_rad_from_direction(rays[i - 1].direction())
        angle = min(angle1 - angle0, np.pi - (angle1 - angle0))
        if i == 1:  # first iteration
            print(f'First Angle: {math.degrees(angle0)}, Distance: {prev_distance}')
            angle_distance_list.append((math.degrees(angle0), prev_distance))
        print(f'Angle: {math.degrees(angle)}, Distance: {curr_distance}')

        angle_distance_list.append((math.degrees(angle), curr_distance))
        if i == len(rays) - 2:
            last_p = np.zeros(2)
            last_p[0] = last_point.x().to_double()
            last_p[1] = last_point.y().to_double()
            distance = np.linalg.norm(last_p - p2)
            angle2 = get_rad_from_direction(rays[i + 1].direction())
            angle = min(angle2 - angle1, np.pi - (angle2 - angle1))
            print(f'Last Angle: {math.degrees(angle)}, Distance: {distance}')
            angle_distance_list.append((math.degrees(angle), distance))
    return angle_distance_list


# -------------------------------- class PathSection -----------------------------
class PathSection:
    def __init__(self, ker_element):
        """contains all the info that the robot needs to move that section"""
        self.KerElement = ker_element

        self.time = 0
        self.distance = 0
        self.mid_fraction = 0.5
        self.part1_dis = 0
        self.part2_dis = 0

        self.isCircle = False
        self.arc_angle = 0
        self.radius = 0

        self.isGlide = False  # if true, means speed is changing in this segment
        self.intervals = 20
        # start and end speeds should be the same if this is not glide motion:
        self.speed_start = 0
        self.speed_middle = 0
        self.speed_end = 0
        self.full_acceleration = 0 # for segments - acceleration considering only the start and end speeds

        self.angle_start = 0  # angle at start of movement. for straight segment this is simply its angle
        self.angle_end = 0  # angle at the end of movement. for segments, it should be the same as angke_start

        self.should_stop = True  # timeout == sleep time ---> full stop at the end of movement.
        # if sleep time is less than timeout, movement will continue (without force)
        self.is_first_movement = False
        self.is_last_movement = False

        # --- circle: ---
        if type(ker_element) is Circle_2:
            self.isCircle = True
            self.radius = math.sqrt(ker_element.squared_radius().to_double())
            self.speed_start = self.speed_end = self.get_max_tangential_speed()

        # --- segment: ---
        elif type(ker_element) is Segment_2:
            self.isGlide = True  # if true, means speed is changing in this segment
            self.distance = math.sqrt(ker_element.squared_length().to_double())
            self.part1_dis = self.mid_fraction * self.distance
            self.part2_dis = self.distance - self.part1_dis
            self.angle_start = self.angle_end = math.degrees(get_rad_from_direction(ker_element.direction()))
        else:
            raise "ker element is neither Circle_2 not Segment_2"

    def get_max_tangential_speed(self):
        if not self.isCircle:
            raise "trying to calculate tangential speed of non circle section"
        empiric_v = 0.5
        empiric_r = 0.6
        max_center_a = (empiric_v * empiric_v) / empiric_r
        max_tangential_speed = math.sqrt(max_center_a * self.radius)
        return max_tangential_speed


def get_rad_from_direction(direction: Direction_2):
    return math.atan2(direction.dy().to_double(), direction.dx().to_double())

def get_linear_acceleration(start_speed, end_speed, seg_len):
    return ((end_speed**2) - (start_speed**2)) / (2 * seg_len)

def get_max_seg_start_speed(end_speed, max_linear_a, seg_len):
    return math.sqrt((end_speed ** 2) - (2 * max_linear_a * seg_len))

# todo del:
def print_smooth_path(smooth_path):
    from smooth_path import get_angle_of_point
    print("\nsmooth path: ")
    for i in range(len(smooth_path)):
        if type(smooth_path[i]) is Ker.Circle_2:
            c = smooth_path[i]
            r = math.sqrt(c.squared_radius().to_double())
            prev_seg = smooth_path[i-1]
            next_seg = smooth_path[i+1]
            arc_source_angle = get_angle_of_point(c, prev_seg.target())
            arc_target_angle = get_angle_of_point(c, next_seg.source())
            # print(f"circle_{i} -- center: {c.center()} -- radius: {r} -- source_point: {prev_seg.target()} -- target_point: {next_seg.source()}"
            #       f" \n-- start_angle: {np.rad2deg(arc_source_angle)} -- end_angle: {np.rad2deg(arc_target_angle)} -- orient: {c.orientation()}\n")
            print(f"circle: {c}")
        else:
            seg = smooth_path[i]
            print(f"seg: {seg}")

# todo del:
def print_robot_path(robot_path):
    print("\nrobot path: ")
    for i in range(len(robot_path)):
        item = robot_path[i]
        if type(item.KerElement) is Ker.Circle_2:
            c = item.KerElement
            print(f"circle: ,        r: {round(item.radius, 2)}       v: {round(item.speed_start, 2)}")
        else:
            seg = item.KerElement
            print(f"seg: ,        len: {round(item.distance,2)}       v0: {round(item.speed_start,2)},   v_mid: {round(item.speed_middle,2)},     vf: {round(item.speed_end,2)}")


# ------------------------------- parse_path2: -------------------------------------------
def parse_path2(smooth_path):
    from smooth_path import get_angle_of_point
    """ assume path is list of segments and circles that connect each segment to the other
        speed/acceleration should be determined afterwards!
        """
    # get empirical max linear acceleration:
    empiric_start_speed = 0.9
    empiric_end_speed = 0.1
    empiric_seg_len = 0.2
    max_linear_a = get_linear_acceleration(empiric_start_speed, empiric_end_speed, empiric_seg_len)

    path_for_robot = []
    for i in range(len(smooth_path)-2, 0, -1):

        if type(smooth_path[i]) is Circle_2:

            # current circle:
            curr_circle = smooth_path[i]
            prev_seg = smooth_path[i - 1]
            next_seg = smooth_path[i + 1]
            circle_sec = PathSection(curr_circle)
            circle_sec.angle_start = get_angle_of_point(curr_circle, prev_seg.target())
            circle_sec.angle_end = get_angle_of_point(curr_circle, next_seg.source())
            circle_sec.arc_angle = circle_sec.angle_end - circle_sec.angle_start

            # next segment:
            next_seg_sec = PathSection(smooth_path[i + 1])
            if next_seg_sec.distance == 0:
                # change the current circle speed to be equal to the next circle:
                circle_sec.speed_start = circle_sec.speed_end = next_circle_sec.speed_start
                next_circle_sec = circle_sec
                path_for_robot = [circle_sec, next_seg_sec] + path_for_robot
                continue
            next_seg_sec.speed_start = circle_sec.speed_end
            # last segment:
            if i == len(smooth_path)-2:
                next_seg_sec.is_last_movement = True
            next_seg_sec.speed_end = 0 if next_seg_sec.is_last_movement else next_circle_sec.speed_start
            next_seg_sec.full_acceleration = get_linear_acceleration(next_seg_sec.speed_start, next_seg_sec.speed_end, next_seg_sec.distance)

            # full acceleration accedes max acceleration:
            if next_seg_sec.full_acceleration < max_linear_a:
                max_seg_start_speed = get_max_seg_start_speed(next_seg_sec.speed_end, max_linear_a, next_seg_sec.distance)
                next_seg_sec.speed_start = max_seg_start_speed
                next_seg_sec.speed_middle = (next_seg_sec.speed_start + next_seg_sec.speed_end) * next_seg_sec.mid_fraction
                circle_sec.speed_start = circle_sec.speed_end = max_seg_start_speed
            # set higher middle speed to segment:
            else:
                next_seg_sec.speed_middle = get_max_seg_start_speed(next_seg_sec.speed_end, max_linear_a, next_seg_sec.part2_dis)

            # saving pointer to the current circle section for next iteration:
            next_circle_sec = circle_sec

            # append current circle and next segment sections at the beginning of the path:
            path_for_robot = [circle_sec, next_seg_sec] + path_for_robot

    # first segment section:
    first_seg_sec = PathSection(smooth_path[0])
    first_seg_sec.is_first_movement = True
    first_seg_sec.speed_start = 0
    first_seg_sec.speed_end = next_circle_sec.speed_start
    first_seg_sec.speed_middle = (first_seg_sec.speed_start + first_seg_sec.speed_end) * first_seg_sec.mid_fraction
    first_seg_sec.full_acceleration = get_linear_acceleration(first_seg_sec.speed_start, first_seg_sec.speed_end, first_seg_sec.distance)
    path_for_robot = [first_seg_sec] + path_for_robot

    # todo del:
    # print_smooth_path(smooth_path)
    # print_robot_path(path_for_robot)

    return path_for_robot
