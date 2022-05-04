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


