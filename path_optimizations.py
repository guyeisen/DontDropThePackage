import math
from typing import List

import numpy as np

from discopygal.bindings import *
from discopygal.solvers import PathPoint


def get_segment(points:List[PathPoint]):
    """
    getting segments made by first and last point of points
    """
    start = points[0].location
    end = points[-1].location
    segment = Segment_2(start, end)
    return segment


def get_max_distance_and_index(points:List[PathPoint]):
    """
    getting tuple of (max index, max distance) of a point in points from the segment they create
    """
    segment = get_segment(points)
    max_index = 0
    max_distance = 0
    for i in range(len(points)-1):
        distance = Ker.squared_distance(segment,points[i].location).to_double()
        if distance > max_distance:
            max_distance = distance
            max_index = i
    max_distance = math.sqrt(max_distance)
    # print(f"max_distance: {max_distance}")
    # print(f"segment length: {math.sqrt(segment.squared_length().to_double())}")
    return max_index, max_distance

def validate_points(points:List[PathPoint],collision_detector):
    """
    returns true if segment created by points is valid
    """
    seg = get_segment(points)
    return collision_detector.is_edge_valid(seg)

def dive_deeper(points:List[PathPoint], max_index:int, collision_detector):
    """
    helper method for simplifying the douglas peuker algorithm
    """
    reduced_left = douglas_peuker(points[:max_index], collision_detector)
    if len(reduced_left) == 2:
        reduced_left = reduced_left if validate_points(reduced_left, collision_detector) else douglas_peuker(points[:max_index-1],collision_detector)+[points[max_index]]

    reduced_right = douglas_peuker(points[max_index:], collision_detector)
    if len(reduced_right) == 2:
        reduced_right = reduced_right if validate_points(reduced_right,collision_detector) else [points[max_index]]+douglas_peuker(points[max_index+1:], collision_detector)

    mid = douglas_peuker([reduced_left[-1], points[max_index], reduced_right[1]], collision_detector)
    if len(mid) == 2:#means it got reduced
        print(f"mid is len 2")
        if validate_points(mid, collision_detector):
            return reduced_left + reduced_right[1:]
    return reduced_left + reduced_right



def douglas_peuker(points:List[PathPoint],collision_detector, epsilon=0.7):
    """
    returns optimaized list of PathPoint according to douglas peuker algorithm.
    considering collision detection
    recursive function
    """
    from smooth_path import get_circle

    # print(f"I GOT HERE DOUGLAS ")
    if len(points) < 3:
        print([p.location for p in points])
        return points
    # print(f"len(points) = {len(points)}")

    max_index, max_distance = get_max_distance_and_index(points)

    if max_distance >= epsilon and max_index > 0:
        if len(points) == 3:
            return points
        result = dive_deeper(points, max_index, collision_detector)
        print([p.location for p in result])
        return result

    else:
        segment = get_segment(points)
        if collision_detector.is_edge_valid(segment):
            print(f"removed {len(points)-2} points")
            print([points[0].location,points[-1].location])
            return [points[0],points[-1]]
        else:
            if len(points)==3:
                print(f"I GOT A THREE THAT I WANT BUT CANT SMOOTHEN!. problematic: {points[1].location}")
                radius = math.sqrt(
                get_circle(points[0].location, points[1].location, points[2].location).squared_radius().to_double())
                get_max_distance_and_index(points)
                print(f"RADIUS IS {radius}m")
                # if radius > 9:
                #     print(f"RADIUS IS {radius}m SMOOTHING WITH FORCE:")
                #     return [points[0],points[-1]]
                print([p.location for p in points])
                return points
            print("wanted to remove but it was intersecting, performing another recursion")
            result = dive_deeper(points,max_index, collision_detector)
            print([p.location for p in result])
            return result



def parse_path(rays: List[Ker.Ray_2], last_point: Point_2):
    """
    NOT IN USE: parsing path to poligonal movement

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
        # start and end speeds should be the same if this is not glide motion:
        self.speed_start = 0
        self.speed_middle = 0
        self.speed_end = 0
        self.full_acceleration = 0 # for segments - acceleration considering only the start and end speeds

        self.angle_start = 0  # angle at start of movement. for straight segment this is simply its angle
        self.angle_end = 0  # angle at the end of movement. for segments, it should be the same as angke_start

        self.should_stop = False  # timeout == sleep time ---> full stop at the end of movement.
        # if sleep time is less than timeout, movement will continue (without force)
        self.is_first_movement = False
        self.is_last_movement = False

        # --- circle: ---
        if type(ker_element) is Circle_2:
            self.isCircle = True
            self.radius = math.sqrt(ker_element.squared_radius().to_double())

        # --- segment: ---
        elif type(ker_element) is Segment_2:
            self.isGlide = True  # if true, means speed is changing in this segment
            self.distance = math.sqrt(ker_element.squared_length().to_double())
            self.part1_dis = self.mid_fraction * self.distance
            self.part2_dis = self.distance - self.part1_dis
            self.angle_start = self.angle_end = math.degrees(get_rad_from_direction(ker_element.direction()))
        else:
            raise Exception("ker element is neither Circle_2 not Segment_2")


def get_rad_from_direction(direction: Direction_2):
    return math.atan2(direction.dy().to_double(), direction.dx().to_double())


def get_max_tangential_speed(max_center_a, r):
    return math.sqrt(max_center_a * r)


def get_linear_acceleration(start_speed, end_speed, seg_len):
    if seg_len == 0:
        return 0
    return ((end_speed**2) - (start_speed**2)) / (2 * seg_len)


def update_linear_acceleration(seg_sec: PathSection):
    seg_sec.full_acceleration = get_linear_acceleration(seg_sec.speed_start, seg_sec.speed_end, seg_sec.distance)


def get_max_seg_start_speed(end_speed, min_linear_d, seg_len):
    return math.sqrt((end_speed ** 2) - (2 * min_linear_d * seg_len))


def get_max_seg_end_speed(start_speed, max_linear_a, seg_len):
    return math.sqrt((start_speed ** 2) + (2 * max_linear_a * seg_len))


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
            part1_intervals = get_intervals_num(item.part1_dis, item.speed_start, item.speed_middle)
            part2_intervals = get_intervals_num(item.part2_dis, item.speed_middle, item.speed_end)
            print(f"        seg: ,        len: {round(item.distance,2)}       v0: {round(item.speed_start,2)},   v_mid: {round(item.speed_middle,2)},     vf: {round(item.speed_end,2)}     part1, part2 intervals: {part1_intervals}, {part2_intervals}")


# todo del:
def get_intervals_num(distance, start_speed, end_speed, max_speed_jump=0.01, max_interval_len=0.03):
    intervals_num = int(math.ceil(math.fabs(end_speed - start_speed) / max_speed_jump) + 1)
    if distance / intervals_num < max_interval_len:
        intervals_num = int(distance // max_interval_len)
    if start_speed != end_speed and intervals_num < 2:
        intervals_num = 2
    return intervals_num


def _calc_angle_rad(p1: Point_2, p2: Point_2):
    dx = p2.x().to_double() - p1.x().to_double()
    dy = p2.y().to_double() - p1.y().to_double()
    return math.atan2(dy, dx)

def _get_Direction_2(p1: Point_2, p2: Point_2):
    dx = p2.x() - p1.x()
    dy = p2.y() - p1.y()
    return Direction_2(dx, dy)


def gen_robot_path_from_points(points):
    #robot_path = optimize_path(points) #CHECK WITH AVIGAIL WHAT IM GETTING BACK. ASSUMING list of points, each telling me what they represent (start/end of segment or arc)
    #robot_path = self.points
    #self.points[0].location.x().to_double()
    #Ray_2(self.points[0].location, Direction_2(self.points[0].location.x(),self.points[0].location.y()))
    print(f"Calculated {len(points)} points")
    rays = [Ker.Ray_2(points[i].location, _get_Direction_2(points[i].location, points[i + 1].location)) for i in range(len(points) - 1)]
    return parse_path(rays, points[len(points)-1].location)


# ---------------- get_circles_sequence: --------------------------------
def get_circles_sequence(start_i, smooth_path, last_seg_sec : PathSection, max_centripetal_acceleration):
    ''' returns a list of PathSections of circles that have segments with no length in between, and ends with a normal segment.
    Additionally it updates the previous segment section end speed and acceleration fields'''
    from smooth_path import get_angle_of_point

    i = start_i
    cir_sec = PathSection(smooth_path[i])
    cir_sec.speed_start = cir_sec.speed_end = get_max_tangential_speed(max_centripetal_acceleration, cir_sec.radius)
    prev_seg_sec = PathSection(smooth_path[i-1])
    next_seg_sec = PathSection(smooth_path[i+1])

    # add angles:
    cir_sec.angle_start = get_angle_of_point(cir_sec.KerElement, prev_seg_sec.KerElement.target())
    cir_sec.angle_end = get_angle_of_point(cir_sec.KerElement, next_seg_sec.KerElement.source())
    cir_sec.arc_angle = cir_sec.angle_end - cir_sec.angle_start

    cir_sec_seq = [cir_sec, next_seg_sec]
    seq_speed = cir_sec.speed_start
    # creating a circles sequence that ends with not zero segment
    while next_seg_sec.distance == 0:
        i+=2
        cir_sec = PathSection(smooth_path[i])
        cir_sec.speed_start = cir_sec.speed_end = get_max_tangential_speed(max_centripetal_acceleration, cir_sec.radius)
        prev_seg_sec = PathSection(smooth_path[i - 1])
        next_seg_sec = PathSection(smooth_path[i + 1])

        # add angles:
        cir_sec.angle_start = get_angle_of_point(cir_sec.KerElement, prev_seg_sec.KerElement.target())
        cir_sec.angle_end = get_angle_of_point(cir_sec.KerElement, next_seg_sec.KerElement.source())
        cir_sec.arc_angle = cir_sec.angle_end - cir_sec.angle_start

        # update seq_speed:
        if cir_sec.speed_start < seq_speed:
            seq_speed = cir_sec.speed_start
        cir_sec_seq = cir_sec_seq + [cir_sec, next_seg_sec]

    # adding the speeds to the sequence circles:
    for sec in cir_sec_seq:
        if type(sec.KerElement) is Circle_2:
            sec.speed_start = seq_speed
            sec.speed_end = seq_speed

    # adding speeds to the previous seg_sec and the last "non-zero" segment section:
    last_seg_sec.speed_end = seq_speed
    cir_sec_seq[-1].speed_start = seq_speed
    update_linear_acceleration(last_seg_sec)

    return cir_sec_seq


# ------------------------------------- fix_linear_accelerations -------------------------------------
def fix_linear_accelerations(robot_path, max_linear_acceleration):
    prev_seg_has_new_end_speed = False
    for sec in robot_path:
        if type(sec.KerElement) is Segment_2 and sec.distance != 0:
            seg_sec = sec
            if prev_seg_has_new_end_speed:
                seg_sec.speed_start = new_speed
                update_linear_acceleration(seg_sec)
            if seg_sec.full_acceleration > max_linear_acceleration:
                new_speed = seg_sec.speed_end = get_max_seg_end_speed(seg_sec.speed_start, max_linear_acceleration, seg_sec.distance)
                prev_seg_has_new_end_speed = True
            else:
                prev_seg_has_new_end_speed = False
        elif type(sec.KerElement) is Circle_2:
            cir = sec
            if prev_seg_has_new_end_speed:
                cir.speed_start = cir.speed_end = new_speed


# ----------------------------------------- fix_linear_decelerations: ------------------------------
def fix_linear_decelerations(robot_path, min_linear_deceleration):
    next_seg_has_new_end_speed = False
    for sec in robot_path[::-1]:
        if type(sec.KerElement) is Segment_2 and sec.distance != 0:
            seg_sec = sec
            if next_seg_has_new_end_speed:
                seg_sec.speed_end = new_speed
                update_linear_acceleration(seg_sec)
            if seg_sec.full_acceleration < min_linear_deceleration:
                new_speed = seg_sec.speed_start = get_max_seg_start_speed(seg_sec.speed_end, min_linear_deceleration, seg_sec.distance)
                next_seg_has_new_end_speed = True
            else:
                next_seg_has_new_end_speed = False
        elif type(sec.KerElement) is Circle_2:
            cir = sec
            if next_seg_has_new_end_speed:
                cir.speed_start = cir.speed_end = new_speed


# ------------------------------ add_optimal_middle_speeds: ---------------------------------------
def add_optimal_middle_speeds(robot_path, max_linear_acceleration, min_linear_deceleration):
    for sec in robot_path:
        if type(sec.KerElement) is Segment_2 and sec.distance != 0:
            seg_sec = sec
            seg_sec.speed_middle = min(get_max_seg_end_speed(seg_sec.speed_start, max_linear_acceleration, seg_sec.part1_dis),
                                       get_max_seg_start_speed(seg_sec.speed_end, min_linear_deceleration, seg_sec.part2_dis))


# ------------------------------- parse_path2: -------------------------------------------
def parse_path2(smooth_path, max_linear_acceleration, min_linear_deceleration, max_centripetal_acceleration):
    from smooth_path import get_angle_of_point
    """ assume path is list of segments and circles that connect each segment to the other
        speed/acceleration should be determined afterwards!
        """

    # print_smooth_path(smooth_path)  # todo del

    # add first seg_sec:
    first_seg_sec = PathSection(smooth_path[0])
    first_seg_sec.is_first_movement = True
    first_seg_sec.speed_start = 0
    robot_path = [first_seg_sec]

    # loop1 -  adds the sections to robot_path, considering the circles sequences:
    i = 1
    prev_seg_sec = first_seg_sec
    while i < len(smooth_path):
        cir_seq = get_circles_sequence(i, smooth_path, prev_seg_sec, max_centripetal_acceleration)
        robot_path = robot_path + cir_seq
        prev_seg_sec = cir_seq[-1]
        i += len(cir_seq)
    # last section:
    last_seg_sec = prev_seg_sec
    last_seg_sec.is_last_movement = True
    last_seg_sec.speed_end = 0
    update_linear_acceleration(last_seg_sec)

    # print("\nafter loop1:") # todo del
    # print_robot_path(robot_path) # todo del

    # loop2 - fix linear accelerations:
    fix_linear_accelerations(robot_path, max_linear_acceleration)

    # print("\nafter loop2 - acceleration fix:") # todo del
    # print_robot_path(robot_path) # todo del

    # loop3 - fix linear decelerations:
    fix_linear_decelerations(robot_path, min_linear_deceleration)

    # print("\nafter loop3 - deceleration fix:") # todo del
    # print_robot_path(robot_path) # todo del

    # loop4 - add optimal middle speeds:
    add_optimal_middle_speeds(robot_path, max_linear_acceleration, min_linear_deceleration)

    # print("\nafter loop4 - adding mid speeds:") # todo del
    # print_robot_path(robot_path) # todo del

    # todo del:
    # print_smooth_path(smooth_path) # todo del
    print_robot_path(robot_path) # todo del

    return robot_path
