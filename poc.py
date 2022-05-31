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
        self._gen_path(env)

    def _calc_angle_rad(self, p1:Point, p2:Point):
        dx = p2.x - p1.x
        dy = p2.y - p1.y
        return math.atan2(dy, dx)

    def _gen_path(self,env: RobotEnvironment, granularity=20):
        length = env.length
        X = np.sort(np.random.uniform(low=0, high=length, size=granularity))
        self.points = [Point(x, math.sin(x)) for x in X]
        self.points.insert(0,Point(0, length/2))
        self.points.append(Point(length, math.sin(length)))
        points = self.points

        self.rays = [Ray(points[i], self._calc_angle_rad(points[i], points[i+1])) for i in range(len(points)-1)]



def generate_environment(base_length=2, scale=1):
    """
    creates enivoroment for robot (including its path inside as env.path
    """
    env = RobotEnvironment(base_length, scale)
    env.path = RobotPath(env)
    return env

def _calc_robot_angle(angle1, angle2):
    pass

def parse_path(path:RobotPath):
    directed_segments = []
    for i in range(len(path.rays)-1):
        p1 = [path.rays[i].source.x, path.rays[i].source.y]
        p2 = [path.rays[i+1].source.x, path.rays[i+1].source.y]
        distance = np.linalg.norm(p2-p1)
        angle = min(np.pi, 2*np.pi - (path.rays[i].angle + path.rays[i+1].angle))
        directed_segments.append((angle,distance))
    return directed_segments

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