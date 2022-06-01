import math
import numpy as np

class Point:
    def __init__(self,x ,y):
        self.x = x
        self.y = y

class Ray:
    def __init__(self, source, angle):
        self.source = source
        self.angle = angle

class RobotEnvironment:

    def __init__(self,base_length=2, scale=1):
        self.base_length = base_length
        self.scale = scale
        self.length = self.base_length*self.scale
        self.points = []
        self.segments = []
        self.path = None
        self._gen_env()


    def _gen_env(self):
        self.points.append(Point(0, 0))
        self.points.append(Point(0, self.length))
        self.points.append(Point( self.length,  self.length))
        self.points.append(Point( self.length, 0))
        # self.segments.append(Segment_2(p0, p1))
        # self.segments.append(Segment_2(p1, p2))
        # self.segments.append(Segment_2(p2, p3))
        # self.segments.append(Segment_2(p3, p0))

env = RobotEnvironment()

import numpy as np
class RobotPath:
    def __init__(self, env):
        self.points = []
        self.rays = []
        self.path_for_robot = []
        #self._gen_path(env)
        self._gen_arc_path_sin(env)
    def _calc_angle_rad(self, p1:Point, p2:Point):
        dx = p2.x - p1.x
        dy = p2.y - p1.y
        return math.atan2(dy, dx)

    def _gen_arc_path_sin(self, env:RobotEnvironment, granularity=20):
        length = env.length
        # X = np.sort(np.random.uniform(low=0, high=length, size=granularity))

        X1 = np.linspace(start=0, stop=0.3, num=int(granularity/4))
        X2 = np.linspace(start=0.74, stop=1.344, num=int(granularity/3))
        X3 = np.linspace(start=1.79, stop=length, num=int(granularity/4))

        self.points1 = [Point(x, math.sin(3 * x) / 2 + 1) for x in X1]
        self.points2 = [Point(x, math.sin(3 * x) / 2 + 1) for x in X2]
        self.points3 = [Point(x, math.sin(3 * x) / 2 + 1) for x in X3]

        self.points1.insert(0, Point(0, length / 2))
        self.points3.append(Point(length, math.sin(length)))
        points1 = self.points1
        points2 = self.points2
        points3 = self.points3

        self.rays1 = [Ray(points1[i], self._calc_angle_rad(points1[i], points1[i + 1])) for i in range(len(points1) - 1)]
        self.rays2 = [Ray(points2[i], self._calc_angle_rad(points2[i], points2[i + 1])) for i in range(len(points2) - 1)]
        self.rays3 = [Ray(points3[i], self._calc_angle_rad(points3[i], points3[i + 1])) for i in range(len(points3) - 1)]
        self.path_for_robot1 = parse_path(self.rays1)
        self.path_for_robot2 = parse_path(self.rays2)
        self.path_for_robot3 = parse_path(self.rays3)

    def _gen_path(self,env: RobotEnvironment, granularity=20):
        length = env.length
        #X = np.sort(np.random.uniform(low=0, high=length, size=granularity))

        X = np.linspace(start=0, stop=length, num=granularity )
        self.points = [Point(x, math.sin(3*x)/2+1) for x in X]
        self.points.insert(0,Point(0, length/2))
        self.points.append(Point(length, math.sin(length)))
        points = self.points

        self.rays = [Ray(points[i], self._calc_angle_rad(points[i], points[i+1])) for i in range(len(points)-1)]
        self.path_for_robot = parse_path(self.rays)


def generate_environment(base_length=2, scale=1):
    """
    creates enivoroment for robot (including its path inside as env.path
    """
    env = RobotEnvironment(base_length, scale)
    env.path = RobotPath(env)
    return env

def _calc_robot_angle(angle1, angle2):
    pass

def to_degrees(rad):
    return 180 * rad / math.pi


def parse_path(rays):
    """
    returns list of tuples: (starting angle, time needed to drive that angle)
    """
    angle_time_list = []
    p1 = np.zeros(2)
    p2 = np.zeros(2)
    for i in range(1, len(rays)-1):
        p1[0] = rays[i].source.x
        p1[1]= rays[i].source.y
        p2[0] = rays[i+1].source.x
        p2[1]= rays[i+1].source.y
        #if p2[0]
        distance = np.linalg.norm(p2-p1)
        #time_for_seg = time_per_length(distance)
        angle = min(rays[i].angle - rays[i - 1].angle, np.pi - (rays[i].angle - rays[i-1].angle))
        print(f'Angle: {to_degrees(angle)}, Distance: {distance}')
        angle_time_list.append((to_degrees(angle), distance))
    return angle_time_list

env = generate_environment()
robot_path = env.path.path_for_robot

def generate_path(env, pointA, pointB):
    """
    given the enviroment produces path from pointA to pointB
    """
    pass


def choose_rays():
    pass



def intersection(ray1, ray2):
    """
    find intersection points between the 2 vectors, taking in mind that vec1 is the origin point.
    :param ray1: origin point
    :param ray2: destination point
    :return: intersection between 2 vectors
    """
    pass

def points_to_segs(points):
    pass

def rays_to_points(rays):
    pass

def rays_to_bezier(rays):
    """
    find intersections between each 2 successive rays. overall we will get 2n-1 points.
    those points will be sent to bezier calculations.
    :param lines: list of n Line_2 known to be safe for movement at t
    :return: bezier curve of degree 2n
    """
    pass


def time_per_length(length, v_const=70, t_const=1.5, u=0.6):
    """
    compute the time needed for Aliza to drive length meters
    given that Aliza drives our unit distance of 0.6 meters
    with constant velocity of 70 and 1.5 time
    """
    return t_const * (length/u)


