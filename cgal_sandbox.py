import importlib
import math

import CGALPY_add_dlls
import CGALPY_kerEpec_aos2ArrSeg_bso2_pol2 as CGALPY

#from CGALPY.Ker import *
#from Ker import *
#from Aos2 import *
Aos2 = CGALPY.Aos2
Ker = CGALPY.Ker

Segment_2 = Ker.Segment_2
Ray_2 = Ker.Ray_2
Vector_2 = Ker.Vector_2
Arrangement_2 = Aos2.Arrangement_2
Point_2 = Arrangement_2.Geometry_traits_2.Point_2
Curv = Arrangement_2.Geometry_traits_2.Curve_2

#CGALPY1 = importlib.import_module('CGALPY')
# from CGALPY import KERNEL
# from CGAL.CGAL_kernel import *

p1 = Point_2(1,1)
p2 = Point_2(3,3)
cir  = Curv(p1, 4)
seg = Segment_2(p1, p2)
seg1 = Segment_2(p1, p2)
#Ker.do_intersect(seg,seg1)
print("seg is:", seg)
print("seg min: ", seg.min())
print("opposite to seg: ", seg.opposite())
print("1.5x1.5 is on seg? - ", seg.has_on(Point_2(1.5, 1.5)))
print("is seg degenerate? - ", seg.is_degenerate())
print("is p2 degenerate? - ", Segment_2(p2,p2).is_degenerate())
print(f"DIRECTION {seg.direction()}")
print(f"squared_length {seg.squared_length()}")


class RobotEnvironment:

    def __init__(self,base_length=2, scale=1):
        self.base_length = base_length
        self.scale = scale
        self.length = self.base_length*self.scale
        self.segments = []
        self._gen_env()


    def _gen_env(self):
        p0 = Point_2(0, 0)
        p1 = Point_2(0, self.length)
        p2 = Point_2( self.length,  self.length)
        p3 = Point_2( self.length, 0)
        self.segments.append(Segment_2(p0, p1))
        self.segments.append(Segment_2(p1, p2))
        self.segments.append(Segment_2(p2, p3))
        self.segments.append(Segment_2(p3, p0))

env = RobotEnvironment()

import numpy as np
class RobotPath:
    def __init__(self, env):
        self.points = []
        self.rays = []
        self._gen_path(env)

    def _gen_path(self,env: RobotEnvironment, granularity=10):
        length = env.length
        X = np.sort(np.random.uniform(low=0, high=length, size=granularity))
        self.points = [Point_2(x, math.sin(x)) for x in X]
        self.points.insert(0,Point_2(0, length/2))
        self.points.append(Point_2(length, math.sin(length)))
        points = self.points
        self.rays = [Ray_2(points[i], Vector_2(points[i], points[i+1])) for i in range(len(points)-1)]



def generate_environment(base_length=2, scale=1):
    """
    returns arrangement as the enviroment
    points are ordered CW (p0, p1, p2 ,p3)
    """
    path = RobotPath(env)
    return path


def generate_path(env, pointA, pointB):
    """
    given the enviroment produces path from pointA to pointB
    """
    points = []


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
    # inter =
    #return inter

def points_to_segs(points):
    segs = []
    for i in range(len(points)-1):
        p0 = points[i]
        p1 = points[i+1]
        seg = Segment_2(p0, p1)
        segs.append(seg)
    return segs

def rays_to_points(rays):
    points = []
    for i in range(len(rays) - 1):
        inter = intersection(rays[i], rays[i + 1])
        points.append(rays[i].source())
        points.append(inter)
    points.append(rays[-1].source())
    return points

def rays_to_bezier(rays):
    """
    find intersections between each 2 successive rays. overall we will get 2n-1 points.
    those points will be sent to bezier calculations.

    :param lines: list of n Line_2 known to be safe for movement at t
    :return: bezier curve of degree 2n
    """
    points = []
    for i in range(len(rays)-1):
        pass
        inter = intersection(rays[i],rays[i+1])
        points.append(rays[i].source())
        points.append(inter)
    points.append(rays[-1].source())

    #bottom line - after this loop we got points holding all the points we need for the bezier curve. IN ORDER



