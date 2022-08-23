import importlib.util
import json
import math
from inspect import isclass

import os
import sys
import json
import importlib.util
from inspect import isclass

from CGALPY.Ker import Ray_2, Direction_2, Vector_2
from discopygal.gui import Worker
from robot_control import *
from discopygal.bindings import Segment_2, Point_2

from discopygal.solvers import Robot, RobotDisc, RobotPolygon, RobotRod
from discopygal.solvers import Obstacle, ObstacleDisc, ObstaclePolygon, Scene
from discopygal.solvers import PathPoint, Path, PathCollection, Solver


from discopygal.solvers.metrics import Metric, Metric_Euclidean
from discopygal.solvers.nearest_neighbors import NearestNeighbors, NearestNeighbors_sklearn
from discopygal.solvers.samplers import Sampler, Sampler_Uniform
from discopygal.geometry_utils import collision_detection, conversions

from path_optimizations import optimize_path


class EnviromentConfigurations:
    def __init__(self):
        self.writer = None
        self.discopygal_scene = Scene()
        self.scene_path = ""
        self.load_scene()
        self.paths = None
        # Setup solver
        self.solver_class = None
        #self.load_solver()
        self.solver_graph = None
        self.solver_arrangement = None
        self.solver_graph_vertices = [] # gui
        self.solver_graph_edges = [] # gui
        self.solver_from_file()
        self.solve()

    def load_scene(self):
        """
        Load a scene.
        """
        scene_name = "testscene.json"
        with open(scene_name, 'r') as fp:
            d = json.load(fp)
        self.discopygal_scene = Scene.from_dict(d)
        self.clear_paths()


    def clear_paths(self):
        pass

    def load_solver(self):
        pass

    def get_solver_args(self):
        """
        Extract a dict from the dynamically generated GUI arguments (to pass to the solver)
        """
        args = {}
        solver_args = self.solver_class.get_arguments()
        for arg in solver_args:
            _, _, ttype = solver_args[arg]
            args[arg] = ttype(solver_args[arg][1])
        return args

    def solve_thread(self):
        """
        The thread that is run by the "solve" function"
        """
        solver_args = self.get_solver_args()
        #solver.set_verbose(self.writer)
        solver = self.solver_class.from_arguments(solver_args)
        solver.load_scene(self.discopygal_scene)
        self.paths = solver.solve()
        self.solver_graph = solver.get_graph()
        self.solver_arrangement = solver.get_arrangement()

    def solver_from_file(self):
        """
        Loads the solver
        """
        path = "prm.py"
        try:
            spec = importlib.util.spec_from_file_location(path, path)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            cnt = 0
            for obj_name in dir(module):
                obj = getattr(module, obj_name)
                if isclass(obj) and issubclass(obj, Solver) and obj_name != "Solver":
                    self.solver_class = obj
                    globals()[obj_name] = obj
                    cnt += 1
        except Exception as e:
            print("Could not import module", repr(e))


    def solve(self):
        """
        This method is called by the solve button.
        Run the MP solver in parallel to the app.
        """
        if self.solver_class is None:
            print("self.solver_class is None")
            return
        self.solve_thread()




import numpy as np
class RobotPath:
    def __init__(self, points):
        self.points = points
        self.rays = []
        self.path_for_robot = self.gen_robot_path_from_points()
        #self._gen_path(env)
        #self._gen_arc_path_sin(env)


    def _calc_angle_rad(self, p1: Point_2, p2: Point_2):
        dx = p2.x().to_double() - p1.x().to_double()
        dy = p2.y().to_double() - p1.y().to_double()
        return math.atan2(dy, dx)



    def _get_Direction_2(self, p1: Point_2, p2: Point_2):
        dx = p2.x() - p1.x()
        dy = p2.y() - p1.y()
        return Direction_2(dx, dy)

    def gen_robot_path_from_points(self):
        #robot_path = optimize_path(points) #CHECK WITH AVIGAIL WHAT IM GETTING BACK. ASSUMING list of points, each telling me what they represent (start/end of segment or arc)
        #robot_path = self.points
        #self.points[0].location.x().to_double()
        #Ray_2(self.points[0].location, Direction_2(self.points[0].location.x(),self.points[0].location.y()))

        self.rays = [Ray_2(self.points[i].location, self._get_Direction_2(self.points[i].location, self.points[i + 1].location)) for i in range(len(self.points) - 1)]
        return parse_path(self.rays)

def _calc_angle_rad_fromDirection(direction: Direction_2):
    return math.atan2(direction.dy().to_double(), direction.dx().to_double())


def to_degrees(rad):
    return 180 * rad / math.pi


def parse_path(rays: [Ray_2]):
    """
    returns list of tuples: (starting angle, distance)
    """
    angle_distance_list = []
    p1 = np.zeros(2)
    p2 = np.zeros(2)
    for i in range(1, len(rays)-1):
        p1[0] = rays[i].source().x().to_double()
        p1[1]= rays[i].source().y().to_double()
        p2[0] = rays[i+1].source().x().to_double()
        p2[1]= rays[i+1].source().y().to_double()
        #if p2[0]
        distance = np.linalg.norm(p2-p1)
        #time_for_seg = time_per_length(distance)
        angle1 = _calc_angle_rad_fromDirection(rays[i].direction())
        angle0 = _calc_angle_rad_fromDirection(rays[i - 1].direction())
        angle = min(angle1 - angle0, np.pi - (angle1 - angle0))
        print(f'Angle: {to_degrees(angle)}, Distance: {distance}')
        angle_distance_list.append((to_degrees(angle), distance))
    return angle_distance_list



def generate_path(env, pointA, pointB):
    """
    given the enviroment produces path from pointA to pointB
    """
    pass


def run_path(control: RobotControl, path):
    for angle, distance in path:
        print(f'Angle: {angle}, Distance: {distance}')
        control.rotate_and_go(angle, distance)


if __name__ == '__main__':
    control = RobotControl()
    env = EnviromentConfigurations()
    robot_path = RobotPath(env.paths.paths[env.discopygal_scene.robots[0]].points)

    run_path(control, robot_path.path_for_robot)