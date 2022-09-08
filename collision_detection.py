import math

from discopygal.bindings import TPoint, Face, Aos2, X_monotone_curve_2, Arrangement_2, Arr_trapezoid_ric_point_location
from discopygal.solvers import Obstacle, ObstacleDisc, ObstaclePolygon, Robot, RobotDisc, RobotPolygon, RobotRod

# from ..bindings import *
# from . import transform
from discopygal.geometry_utils import transform

from discopygal.bindings import *
from smooth_path import *

EPS = 0.001


class ObjectCollisionDetection(object):
    """
    A class object that handles collision detection of a single object with obstacles.
    The collision detector builds a CGAL arrangement representing the scene and allows to
    (quickly) query the arrangement for collisions.

    :param obstacles: list of obstacles
    :type obstacles: list<class:'discopygal.solvers.Obstacle'>
    :param robot: robot for building the collision detection
    :type robot: class:'discopygal.solvers.Robot'
    """

    def __init__(self, obstaclces, robot):
        self.obstacles = obstaclces
        self.robot = robot

        self.cspace = None
        self.point_location = None
        self.build_cspace()

    def is_point_valid(self, point):
        """
        Check if a point is valid (i.e. not colliding with anything).

        :param point: point to check
        :type point: class:'Ker.Point_2'

        :return: False if point lies in the interior of an obstacle
        :rtype: bool
        """
        point = TPoint(point.x(), point.y())  # convert to traits point
        obj = self.point_location.locate(point)
        f = Face()
        if obj.get_face(f):
            if f.data() > 0:
                return False
        return True

    def is_edge_valid(self, edge):
        """
        Check if a edge is valid (i.e. not colliding with anything).

        :param edge: edge to check
        :type edge: class:'Ker.Segment_2'

        :return: False if edge intersects with the interior of an obstacle
        :rtype: bool
        """
        res = []
        if edge.is_degenerate():
            return True
        Aos2.zone(self.cspace, X_monotone_curve_2(edge.source(), edge.target()), res, self.point_location)
        for obj in res:
            if type(obj) == Face:
                if obj.data() > 0:
                    return False
        return True


    def is_arc_valid_approximated(self, circle, arc_source, arc_target, robot_radius):

        center = circle.center()
        x_center = center.x().to_double()
        y_center = center.y().to_double()
        arc_radius = math.sqrt(circle.squared_radius().to_double())
        arc_source_angle = get_angle_of_point(circle, arc_source)
        arc_target_angle = get_angle_of_point(circle, arc_target)

        # delta angle of segments:
        theta = 2 * math.fabs(math.cos(arc_radius/(arc_radius-robot_radius)))

        # ---- division of arc into segments: -------
        segments_list = []
        seg_source = arc_source
        seg_source_angle = arc_source_angle
        seg_target_angle = seg_source_angle + theta
        condition = (seg_target_angle < arc_target_angle) if circle.orientation() == Ker.COUNTERCLOCKWISE else (seg_target_angle > arc_target_angle)
        sign = 1 if circle.orientation() == Ker.COUNTERCLOCKWISE else -1
        while condition:
            x_seg_target = x_center + (arc_radius * math.cos(seg_target_angle))
            y_seg_target = y_center + (arc_radius * math.sin(seg_target_angle))
            seg_target = Point_2(x_seg_target, y_seg_target)
            seg = Segment_2(seg_source, seg_target)
            segments_list.append(seg)

            # for next iteration:
            seg_source = seg_target
            seg_source_angle = seg_target_angle
            seg_target_angle = seg_source_angle + (sign * theta)
            condition = (seg_target_angle < arc_target_angle) if circle.orientation() == Ker.COUNTERCLOCKWISE else (seg_target_angle > arc_target_angle)

        # add last segment:
        seg = Segment_2(seg_source, arc_target)
        segments_list.append(seg)


        # ----- check if all segments are valid: ------------
        for seg in segments_list:
            if not self.is_edge_valid(seg):
                return False
        return True


    def build_cspace(self):
        """
        Build the Cspace arrangement
        """
        self.cspace = Arrangement_2()
        ubf = self.cspace.unbounded_face()
        ubf.set_data(0)
        if len(self.obstacles) == 0:
            # If not obstacles then the cmap is empty
            self.point_location = Arr_trapezoid_ric_point_location(self.cspace)
            return

        # Overlay single object arrangements
        traits = Arr_face_overlay_traits(lambda x, y: x + y)
        arrangements = [self.cspace]
        for obstacle in self.obstacles:
            arr_obstacle = self.expanded_obstacle_arrangement(obstacle)
            arrangements.append(arr_obstacle)

        for i in range(len(arrangements) - 1):
            res = Arrangement_2()
            Aos2.overlay(arrangements[i], arrangements[i + 1], res, traits)
            self.clean_redundant_halfedges(res)
            arrangements[i + 1] = res

        self.cspace = arrangements[-1]

        # Also build the point location
        self.point_location = Arr_trapezoid_ric_point_location(self.cspace)

    def clean_redundant_halfedges(self, arr):
        """
        Given an arrangement, clear any halfedge that its both incident faces are nonfree.

        :param arr: arrangement to clean
        :type arr: class:'Aos2.Arrangement_2'
        """
        halfedges_to_remove = []
        for halfedge in arr.edges():
            if halfedge.face().data() > 0 and halfedge.twin().face().data() > 0:
                halfedges_to_remove.append(halfedge)
        for halfedge in halfedges_to_remove:
            Aos2.remove_edge(arr, halfedge)

    def expanded_obstacle_arrangement(self, obstacle):
        """
        Given a robot shape and the current obstacle, generate an arrangement
        that contains the (single) expanded obstacle

        :param obstacle: obstacle to expand
        :type obstacle: class:'discopygal.solvers.Obstacle'

        :return: arrangement with expanded obstacle
        :rtype: class:'Aos2.Arrangement_2'
        """
        arr = Arrangement_2()

        if type(self.robot) is RobotDisc:
            # Handle the disc robot case
            if type(obstacle) is ObstaclePolygon:
                ms = MN2.approximated_offset_2(obstacle.poly, FT(self.robot.radius), EPS)
                Aos2.insert(arr, [curve for curve in ms.outer_boundary().curves()])
            elif type(obstacle) is ObstacleDisc:
                expanded_radius = obstacle.radius + self.robot.radius
                circle = Circle_2(obstacle.location, expanded_radius * expanded_radius, Ker.CLOCKWISE)
                Aos2.insert(arr, Curve_2(circle))

        elif type(self.robot) is RobotPolygon:
            minus_robot = Polygon_2(
                [Point_2(-p.x(), -p.y()) for p in self.robot.poly.vertices()]
            )
            if minus_robot.is_clockwise_oriented():
                minus_robot.reverse_orientation()
            if type(obstacle) is ObstaclePolygon:
                if obstacle.poly.is_clockwise_oriented():
                    obstacle.poly.reverse_orientation()
                ms = MN2.minkowski_sum_2(obstacle.poly, minus_robot)
                Aos2.insert(arr, [Curve_2(edge) for edge in ms.outer_boundary().edges()])
                for hole in ms.holes():
                    Aos2.insert(arr, [Curve_2(edge) for edge in hole.edges()])

            if type(obstacle) is ObstacleDisc:
                # Shift the minus robot to the disc location and then expand
                minus_robot = transform.offset_polygon(minus_robot, obstacle.location)
                ms = MN2.approximated_offset_2(minus_robot, obstacle.radius, EPS)
                Aos2.insert(arr, [curve for curve in ms.outer_boundary().curves()])

        elif type(self.robot) is RobotRod:
            pass

        # Set unbounded face and holes as free, and the polygon as occupied
        ubf = arr.unbounded_face()
        ubf.set_data(0)
        invalid_face = next(next(ubf.inner_ccbs())).twin().face()
        invalid_face.set_data(1)
        for ccb in invalid_face.inner_ccbs():
            # Set holes as free
            valid_face = next(ccb).twin().face()
            valid_face.set_data(0)
        return arr


def collide_two_robots(robot1, edge1, robot2, edge2):
    """
    Get two robots and an edge of their movement, and check if at any point
    during their movement they intersect

    :param robot1: first robot
    :type robot1: class:'Robot'
    :param edge1: first robot edge motion
    :type edge1: class:'Ker.Segment_2'
    :param robot2: second robot
    :type robot2: class:'Robot'
    :param edge2: second robot edge motion
    :type edge2: class:'Ker.Segment_2'

    :return: True if robots collide during motion
    :rtype: bool
    """
    if type(robot1) is RobotRod or type(robot2) is RobotRod:
        raise Exception("Rod robot not yet implemented")

    # Without loss of generality, assume robot 1 is expanded and lies in the origin
    # and that robot 2 is a point and the only one that is moving
    arr = Arrangement_2()

    if type(robot1) is RobotDisc and type(robot2) is RobotDisc:
        ms_radius = robot1.radius + robot2.radius
        ms_radius_squared = ms_radius * ms_radius
        circle = Circle_2(Point_2(FT(0), FT(0)), ms_radius_squared, Ker.CLOCKWISE)
        Aos2.insert(arr, Curve_2(circle))
    elif type(robot1) is RobotPolygon and type(robot2) is RobotPolygon:
        minus_robot2 = Polygon_2(
            [Point_2(-p.x(), -p.y()) for p in robot2.poly.vertices()]
        )

        if minus_robot2.is_clockwise_oriented():
            minus_robot2.reverse_orientation()
        if robot1.poly.is_clockwise_oriented():
            robot1.poly.reverse_orientation()
        ms = MN2.minkowski_sum_2(robot1.poly, minus_robot2)
        Aos2.insert(arr, [Curve_2(edge) for edge in ms.outer_boundary().edges()])
        for hole in ms.holes():
            Aos2.insert(arr, [Curve_2(edge) for edge in hole.edges()])
    elif type(robot1) is RobotPolygon and type(robot2) is RobotDisc:
        ms = MN2.approximated_offset_2(robot1.poly, robot2.radius, EPS)
        Aos2.insert(arr, [curve for curve in ms.outer_boundary().curves()])
    elif type(robot1) is RobotDisc and type(robot2) is RobotPolygon:
        ms = MN2.approximated_offset_2(robot2.poly, robot1.radius, EPS)
        Aos2.insert(arr, [curve for curve in ms.outer_boundary().curves()])
    else:
        raise Exception("Not implemented")

    # Subtract robot1's motion from the motion of robot2
    edge_start = Point_2(
        edge2.source().x() - edge1.source().x(),
        edge2.source().y() - edge1.source().y()
    )
    edge_end = Point_2(
        edge2.target().x() - edge1.target().x(),
        edge2.target().y() - edge1.target().y()
    )
    edge = X_monotone_curve_2(edge_start, edge_end)

    point_location = Arr_trapezoid_ric_point_location(arr)

    if edge.source() != edge.target():
        res = []
        Aos2.zone(arr, edge, res, point_location)
        for obj in res:
            if type(obj) == Face:
                if not obj.is_unbounded():
                    # If we intersect with the interior of a face
                    return True
        return False
    else:
        ##################
        # Edge case - both robot don't move but they already intersect!
        ##################

        point = TPoint(edge.source().x(), edge.source().y())
        obj = point_location.locate(point)
        f = Face()
        if obj.get_face(f):
            if not f.is_unbounded():
                # Only if we intersect with the interior of the robot
                return True
        return False


def collide_disc_with_polygon(center, r, polygon):
    """
    Collide (center ,r) disc with CGAL polygon

    :param center: center of the disc to intersect
    :type center: class:'Ker.Point_2'
    :param r: radius of disc to intersect
    :type r: class:'Ker.FT'
    :param polygon: polygon to intersect with
    :type polygon: class:'Pol2.Polygon_2'

    :return: true if disc and polygon intersect
    :rtype: bool
    """
    # Expand the polygon in the radius and convert to arrangement
    minkowski_sum = MN2.approximated_offset_2(polygon, FT(r), EPS)
    arr = Arrangement_2()
    Aos2.insert(arr, [curve for curve in minkowski_sum.outer_boundary().curves()])

    # Set unbounded face and holes as free, and the polygon as occupied
    ubf = arr.unbounded_face()
    ubf.set_data(0)
    invalid_face = next(next(ubf.inner_ccbs())).twin().face()
    invalid_face.set_data(1)
    for ccb in invalid_face.inner_ccbs():
        # Set holes as free
        valid_face = next(ccb).twin().face()
        valid_face.set_data(0)
    point_location = Arr_trapezoid_ric_point_location(arr)

    # Check if the point is free
    point = TPoint(center.x(), center.y())  # convert to Aos2 point (instead of kernel)
    obj = point_location.locate(point)
    f = Face()
    if obj.get_face(f):
        if f.data() > 0:
            return True
        return False

    # If we intersect with something other than a face then we intersect, hence True
    return True


def collide_disc_with_disc(center1, r1, center2, r2):
    """
    Collide (center ,r) disc with (center ,r) disc

    :param center1: center of the first disc to intersect
    :type center1: class:'Ker.Point_2'
    :param r1: radius of first disc to intersect
    :type r1: class:'Ker.FT'
    :param center2: center of the second disc to intersect
    :type center2: class:'Ker.Point_2'
    :param r2: radius of second disc to intersect
    :type r2: class:'Ker.FT'

    :return: true if discs intersect
    :rtype: bool
    """
    # Build arrangement containins the (expanded) disc
    arr = Arrangement_2()
    expanded_radius = r1 + r2
    circle = Circle_2(center1, expanded_radius * expanded_radius, Ker.CLOCKWISE)
    Aos2.insert(arr, Curve_2(circle))
    point_location = Arr_trapezoid_ric_point_location(arr)

    # Check if the point is free
    point = TPoint(center2.x(), center2.y())  # convert to Aos2 point (instead of kernel)
    obj = point_location.locate(point)
    f = Face()
    if obj.get_face(f):
        if not f.is_unbounded():
            return True
        return False

    # If we intersect with something other than a face then we intersect, hence True
    return True


def collide_disc_with_rod(center, r, x, y, a, length):
    """
    Collide (center, r) disc with (x, y, a, length) rod

    :param center: center of the disc to intersect
    :type center: class:'Ker.Point_2'
    :param r: radius of disc to intersect
    :type r: class:'Ker.FT'
    :param x: x of rod
    :type x: class:'Ker.FT'
    :param y: y of rod
    :type y: class:'Ker.FT'
    :param a: angle of rod
    :type a: class:'Ker.FT'
    :param length: length of rod
    :type length: class:'Ker.FT'

    :return: true if disc and segment intersect
    :rtype: bool
    """
    # Build arrangement containins the disc
    arr = Arrangement_2()
    circle = Circle_2(center, r * r, Ker.CLOCKWISE)
    Aos2.insert(arr, Curve_2(circle))
    point_location = Arr_trapezoid_ric_point_location(arr)

    # Convert rod to Segment_2
    r0 = Vector_2(length, FT(0))
    p = Point_2(x, y)
    at = Aff_transformation_2(Rotation(), FT(Gmpq(math.sin(a.to_double()))),
                              FT(Gmpq(math.cos(a.to_double()))), FT(1))
    p0 = p + at.transform(r0)
    segment = Segment_2(p, p0)

    # Check if segment intersects circle
    res = []
    Aos2.zone(arr, X_monotone_curve_2(segment.source(), segment.target()), res, point_location)
    for obj in res:
        if type(obj) == Face:
            if not obj.is_unbounded():
                return True
        else:
            # If we intersect with something other than a face then we intersect, hence True
            return True
    return False

# class SegArcCollisionDetector()