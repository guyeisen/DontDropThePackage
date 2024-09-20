import math
import numpy as np
from discopygal.bindings import *
from collision_detection import *

Segment_2 = Ker.Segment_2
Ray_2 = Ker.Ray_2
Vector_2 = Ker.Vector_2
Arrangement_2 = Aos2.Arrangement_2
Point_2 = Ker.Point_2
Circle_2 = Ker.Circle_2


# -----------------------------------------------------------------
def dis(p1, p2):
    """ Returns the Euclidean distance between two Points on the plane. """
    x1 = p1.x().to_double()
    x2 = p2.x().to_double()
    y1 = p1.y().to_double()
    y2 = p2.y().to_double()
    dx = x2-x1
    dy = y2-y1
    return math.sqrt(dx*dx + dy*dy)


# ---------------------------- get_angle: -------------------------------------
def get_angle(p1, p2, p3):
    """ Returns the angle between the three points, using the definition of dot product. """

    x1 = p1.x().to_double()
    x2 = p2.x().to_double()
    x3 = p3.x().to_double()
    y1 = p1.y().to_double()
    y2 = p2.y().to_double()
    y3 = p3.y().to_double()

    k = (dis(p1, p2)) * (dis(p2, p3))
    if k == 0:
        return 0
    theta = math.acos(((x3 - x2) * (x1 - x2) + (y3 - y2) * (y1 - y2)) / k)
    return theta


# ----------------------------- get_arc_source_and_target: ------------------------------------
def get_arc_source_and_target(p1, p2, p3):
    """ input = three points: segment1_source, segment1_target, segment2_target
        output = arc source and target points
        (the closer point out of p1 and p2 and a point on the other segment that is at the same distance as the closer point from p2)"""
    s1 = Segment_2(p1, p2)
    s2 = Segment_2(p2, p3)

    shorter_s = s1 if s1.squared_length() < s2.squared_length() else s2 # the shorter segment
    len_shorter_s = math.sqrt(shorter_s.squared_length().to_double())
    longer_s = s2 if s1.squared_length() < s2.squared_length() else s1  # the longer segment
    if longer_s == s1:
        longer_s_ray = Ray_2(p2, longer_s.source())
    else:
        longer_s_ray = Ray_2(p2, longer_s.target())
    closer_p = s1.source() if s1.squared_length() < s2.squared_length() else s2.target() # the closer point
    # the intersection of "c" and "longer_s"
    other_p = get_point_on_ray(longer_s_ray, len_shorter_s)

    if shorter_s == s1:
        source = closer_p
        target = other_p
    else:
        source = other_p
        target = closer_p

    return source, target


# ---------------------------- get_angle_of_point: ---------------------------------
def get_angle_of_point(circle, point):
    """ Returns the mathematical angle of a point located on a circle.
    Meaning the angle from the positive x axis to the segment connecting the point to the circle center."""
    center = circle.center()
    xc = center.x().to_double()
    yc = center.y().to_double()
    x = point.x().to_double()
    y = point.y().to_double()
    r = math.sqrt(circle.squared_radius().to_double())

    dy = math.fabs(y-yc)

    if xc < x and yc < y:
        alpha = math.fabs(math.asin(dy/r))
    elif x < xc and yc < y:
        alpha = 0.5 * math.pi + math.fabs(math.acos(dy/r))
    elif x < xc and y < yc:
        alpha = math.pi + math.fabs(math.asin(dy/r))
    else:
        alpha = 1.5 * math.pi + math.fabs(math.acos(dy/r))

    return alpha


# ------------------------ get point on ray: -------------------------------------
def get_point_on_ray(ray, distance):
    """ return a point in a given distance from the beginning of a given ray. """
    source = ray.source()
    x_source = source.x().to_double()
    y_source = source.y().to_double()

    direction = ray.direction()
    dx = direction.dx().to_double()
    dy = direction.dy().to_double()

    squared_slope = (dy/dx)*(dy/dx)

    x_sign = dx/math.fabs(dx)
    y_sign = dy/math.fabs(dy)

    x_dis = x_sign*(math.sqrt((distance**2) / (1+squared_slope)))
    y_dis = y_sign*(math.fabs((dy/dx) * x_dis))

    x = x_source + x_dis
    y = y_source + y_dis

    return Ker.Point_2(x, y)


# ---------------------------- get_circle: -------------------------------------
def get_circle(p1, p2, p3):
    """ Return a circle that is tangent to segments p1-p2 and p2-p3 at the closer point out of p1 and p3"""
    x1 = p1.x().to_double()
    x2 = p2.x().to_double()
    x3 = p3.x().to_double()
    y1 = p1.y().to_double()
    y2 = p2.y().to_double()
    y3 = p3.y().to_double()

    s1 = Segment_2(p1, p2)
    s2 = Segment_2(p2, p3)

    shorter_s = s1 if s1.squared_length() < s2.squared_length() else s2 # the shorter segment
    closer_p = p1 if s1.squared_length() < s2.squared_length() else p3 # the closer point
    x_closer = closer_p.x().to_double()
    len_shorter_s = math.sqrt(shorter_s.squared_length().to_double())

    # the angle between the segments
    theta = get_angle(p1, p2, p3)

    # the circle radius
    r = math.tan(0.5 * theta) * len_shorter_s

    # the angle from shorter_s to the x axis
    alpha = math.acos((x2-x_closer)/len_shorter_s)

    # -- the circle center --
    line_shorter_seg = Ker.Line_2(closer_p, p2)
    perpendicular_line = line_shorter_seg.perpendicular(closer_p)
    if (closer_p == p1 and Ker.right_turn(p1, p2, p3)) or (closer_p == p3 and Ker.left_turn(p1, p2, p3)):
        perpendicular_line = perpendicular_line.opposite()
    ray_to_center = Ker.Ray_2(closer_p, perpendicular_line)
    center = get_point_on_ray(ray_to_center, r)

    # arc direction (Img4)
    direction = Ker.CLOCKWISE if ((x2-x1)*(y3-y2)-(y2-y1)*(x3-x2)) < 0 else Ker.COUNTERCLOCKWISE
    c = Circle_2(center, FT(r*r), direction) # The radius is squared since it allows for much more exact constructions

    return c


# ---------------------------------- smooth_path: ---------------------------------------------------
def get_smooth_path(gui, use_cd=True):
    """ returns a list of segments and circles alternately.
    the circles forms circular arcs that starts/ends at the midpoint of the original segments or closer. """

    robot_radius = 0.25

    # -- get the collision_detector object: --
    robot = gui.discopygal_scene.robots[0]
    prm = gui.solver
    collision_detector: ObjectCollisionDetection = prm.collision_detection[robot]

    res = []

    path = gui.paths_optimized.paths[robot]
    points = path.points  # type points: list<class:`PathPoint`>
    prev_point = points[0].location
    for i in range(1, len(points)-1):
        p1 = points[i-1].location
        p2 = points[i].location
        p3 = points[i+1].location

        p_seg1 = Ker.midpoint(p1, p2)
        p_seg2 = Ker.midpoint(p2, p3)
        c = get_circle(p_seg1, p2, p_seg2)
        arc_source, arc_target = get_arc_source_and_target(p_seg1, p2, p_seg2)

        if use_cd:
            # ---- success on the first attempt: -------
            if collision_detector.is_arc_valid_approximated(c, arc_source, arc_target, robot_radius):
                res.append(Ker.Segment_2(prev_point, arc_source))
                res.append(c)
                prev_point = arc_target
            # ---- binary search for the optimal circle: ---
            else:
                closer_p = p1 if dis(p1,p2) < dis(p2,p3) else p3  # the closer point
                valid_c = valid_arc_source = valid_arc_target = None
                if closer_p == p1:
                    start = p_seg1
                    end = p2
                    for i in range(10):
                        c = get_circle(p_seg1, p2, p_seg2)
                        arc_source, arc_target = get_arc_source_and_target(p_seg1, p2, p_seg2)
                        if collision_detector.is_arc_valid_approximated(c, arc_source, arc_target, robot_radius):
                            end = p_seg1
                            valid_c, valid_arc_source, valid_arc_target = c, arc_source, arc_target
                        else:
                            start = p_seg1
                        p_seg1 = Ker.midpoint(start,end)

                else:  # closer_p is p2
                    start = p2
                    end = p_seg2
                    for i in range(10):
                        c = get_circle(p_seg1, p2, p_seg2)
                        arc_source, arc_target = get_arc_source_and_target(p_seg1, p2, p_seg2)
                        if collision_detector.is_arc_valid_approximated(c, arc_source, arc_target, robot_radius):
                            start = p_seg2
                            valid_c, valid_arc_source, valid_arc_target = c, arc_source, arc_target
                        else:
                            end = p_seg2
                        p_seg2 = Ker.midpoint(start, end)

                if valid_c is None: # didn't found a valid circle in 10 iteration - return circle of radius 0.0001
                    res.append(Ker.Segment_2(prev_point, p2))
                    res.append(Ker.Circle_2(p2, FT(0.0001), Ker.CLOCKWISE))
                    prev_point = p2
                else:
                    res.append(Ker.Segment_2(prev_point, valid_arc_source))
                    res.append(valid_c)
                    prev_point = valid_arc_target
        else:
            # without collision detector
            res.append(Ker.Segment_2(prev_point, arc_source))
            res.append(c)
            prev_point = arc_target

    # append the last segment:
    res.append(Ker.Segment_2(prev_point, points[-1].location))
    return res
