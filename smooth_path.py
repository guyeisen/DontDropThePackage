import math
from discopygal.bindings import *

from collision_detection import *

# Aos2 = CGALPY.Aos2
# Ker = CGALPY.Ker

Segment_2 = Ker.Segment_2
Ray_2 = Ker.Ray_2
Vector_2 = Ker.Vector_2
Arrangement_2 = Aos2.Arrangement_2
Point_2 = Ker.Point_2
Circle_2 = Ker.Circle_2
# Curv = Arrangement_2.Geometry_traits_2.Curve_2

Arrangement_2 = Aos2.Arrangement_2
Curve_2 = Aos2.Traits.Curve_2

# --- functions that can be usefull: ---
# t = Ker.angle(p1, p2, p3) # return enum of OBTUSE/RIGHT/ACUTE
# Ker.midpoint(p1, p2)

# -----------------------------------------------------------------

def dis(p1, p2):
    x1 = p1.x().to_double()
    x2 = p2.x().to_double()
    y1 = p1.y().to_double()
    y2 = p2.y().to_double()
    dx = x2-x1
    dy = y2-y1
    return math.sqrt(dx*dx + dy*dy)

# ---------------------------- get_angle: -------------------------------------

def get_angle(p1,p2,p3):
    """ calculated by the definition of dot product (Img2)"""
    x1 = p1.x().to_double()
    x2 = p2.x().to_double()
    x3 = p3.x().to_double()
    y1 = p1.y().to_double()
    y2 = p2.y().to_double()
    y3 = p3.y().to_double()
    theta = math.acos(((x3 - x2) * (x1 - x2) + (y3 - y2) * (y1 - y2)) / (dis(p1, p2)) * (dis(p2, p3)))
    return theta

# ----------------------------- get_arc_source_and_target: ------------------------------------

def get_arc_source_and_target(p1, p2, p3):
    s1 = Segment_2(p1, p2)
    s2 = Segment_2(p2, p3)

    shorter_s = s1 if s1.squared_length() < s2.squared_length() else s2 # the shorter segment
    len_shorter_s = math.sqrt(shorter_s.squared_length().to_double())
    longer_s = s2 if s1.squared_length() < s2.squared_length() else s1  # the shorter segment
    longer_s_ray = Ray_2(longer_s.source(), longer_s.target())

    closer_p = s1.source() if s1.squared_length() < s2.squared_length() else s2.target() # the closer point
    other_p = longer_s_ray.point(FT(len_shorter_s)) # the intersection of "c" and "longer_s"

    if shorter_s == s1:
        source = closer_p
        target = other_p
    else:
        source = other_p
        target = closer_p

    return source, target

# ---------------------------- get_circle: -------------------------------------

''' return a circle that is tangent to segments p1-p2 and p2-p3 at some points that are closer then p1 and p3 (or equal to them)'''
def get_circle(p1, p2, p3):
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
    y_closer = closer_p.y().to_double()
    len_shorter_s = math.sqrt(shorter_s.squared_length().to_double())

    # the angle between the segments (Img2)
    theta = get_angle(p1, p2, p3)

    # the circle radius (Img1)
    r = math.tan(0.5 * theta) * len_shorter_s

    # the angle from shorter_s to the x axis (Img3)
    alpha = math.acos((x2-x_closer)/len_shorter_s)

    # the circle center (Img3)
    sign_x = 1 if x3-x1 > 0 else -1
    sign_y = 1 if y3-y1 > 0 else -1
    xc = x_closer + sign_x * math.fabs(r * math.cos(alpha - 0.5 * math.pi))
    yc = y_closer + sign_y * math.fabs(r * math.sin(alpha - 0.5 * math.pi))
    center = Point_2(xc, yc)

    # arc direction (Img4)
    direction = Ker.CLOCKWISE if ((x2-x1)*(y3-y2)-(y2-y1)*(x3-x2)) < 0 else Ker.COUNTERCLOCKWISE
    c = Circle_2(center, FT(r*r), direction) # The radius is squared since it allows for much more exact constructions

    return c

# ---------------------------------- smooth_path: ---------------------------------------------------

def smooth_path(path_collection, collision_detector):
    ''' returns a list of segments and circles alternately.
    the circles forms circular arcs that starts/ends at the midpoint of the original segments at most '''
    res = []
    paths_dict = path_collection.paths
    for robot, path in paths_dict:
        points = path.points # type points: list<class:`PathPoint`>
        prev_point = points[0].location
        for i in range(len(points)-3):
            p1 = points[i].location
            p2 = points[i+1].location
            p3 = points[i+2].location

            midpoint_seg1 = Ker.midpoint(p1, p2)
            midpoint_seg2 = Ker.midpoint(p2, p3)

            c = get_circle(midpoint_seg1, p2, midpoint_seg2)
            arc_source, arc_target = get_arc_source_and_target(midpoint_seg1, p2, midpoint_seg2)
                arc_angle = (2 * math.pi) - get_angle(p1, p2, p3)

            if collision_detector.is_arc_valid(c, arc_angle):
                res.append(Ker.segment_2(prev_point, arc_source))
                res.append(c)
            else:
                # todo - find the largest circle that don't collide by binary search
                # midpoint_seg1 = Ker.midpoint(midpoint_seg1, p2)
                # midpoint_seg2 = Ker.midpoint(p2, midpoint_seg2)

            prev_point = arc_target

    return res



# ---------------------------------- main -------------------------------------------------------

if __name__ == '__main__':
    p1 = Point_2(1, 1)
    p2 = Point_2(2, 2)
    p3 = Point_2(3, 2)

    s1 = Segment_2(p1, p2)
    s2 = Segment_2(p2, p3)

    c = get_circle(p1, p2, p3)
    arc_angle = (2 * math.pi) - get_angle(p1, p2, p3)

    shorter_s = s1 if s1.squared_length() < s2.squared_length() else s2 # the shorter segment
    len_shorter_s = math.sqrt(shorter_s.squared_length().to_double())
    longer_s = s2 if s1.squared_length() < s2.squared_length() else s1  # the shorter segment
    longer_s_ray = Ray_2(longer_s.source(), longer_s.target())

    closer_p = p1 if s1.squared_length() < s2.squared_length() else p3 # the closer point
    other_p = longer_s_ray.point(FT(len_shorter_s)) # the intersection of "c" and "longer_s"

    # arc = Circular_arc_2(c, closer_p, other_p)

    print(c.center())
    print(c.squared_radius())

