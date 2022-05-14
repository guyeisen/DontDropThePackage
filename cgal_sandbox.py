import CGALPY_add_dlls
import CGALPY

from CGALPY.Ker import *

# from CGALPY import KERNEL
# from CGAL.CGAL_kernel import *

p1 = Point_2(1,1)
p2 = Point_2(2,2)

seg = Segment_2(p1, p2)
print("seg is:", seg)
print("seg min: ", seg.min())
print("opposite to seg: ", seg.opposite())
print("1.5x1.5 is on seg? - ", seg.has_on(Point_2(1.5, 1.5)))
print("is seg degenerate? - ", seg.is_degenerate())
print("is p2 degenerate? - ", Segment_2(p2,p2).is_degenerate())
print(f"DIRECTION {seg.direction()}")
print(f"squared_length {seg.squared_length()}")

def intersection(ray1, ray2):
    """
    find intersection points between the 2 vectors, taking in mind that vec1 is the origin point.

    :param ray1: origin point
    :param ray2: destination point
    :return: intersection between 2 vectors
    """


def points_to_bezier(rays):
    """
    find intersections between each 2 successive rays. overall we will get 2n-1 points.
    those points will be sent to bezier calculations.

    :param lines: list of n Line_2 known to be safe for movement at t
    :return: bezier curve of degree 2n
    """
    points = []
    for i in range(len(rays)-1):
        inter = intersection(rays[i],rays[i+1])
        # adding those 3 points by order to points
    #bottom line - after this loop we got points holding all the points we need for the bezier curve. IN ORDER



