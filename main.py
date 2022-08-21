import json
import math
from discopygal.gui import Worker

from discopygal.bindings import Segment_2, Point_2

from discopygal.solvers import Robot, RobotDisc, RobotPolygon, RobotRod
from discopygal.solvers import Obstacle, ObstacleDisc, ObstaclePolygon, Scene
from discopygal.solvers import PathPoint, Path, PathCollection, Solver


from discopygal.solvers.metrics import Metric, Metric_Euclidean
from discopygal.solvers.nearest_neighbors import NearestNeighbors, NearestNeighbors_sklearn
from discopygal.solvers.samplers import Sampler, Sampler_Uniform
from discopygal.geometry_utils import collision_detection, conversions

class RobotSetup:
    def __init__(self):
        self.writer = None
        self.discopygal_scene = Scene()
        self.scene_path = ""
        self.load_scene()

        # Setup solver
        self.solver_class = None
        self.load_solver()
        self.solver_graph = None
        self.solver_arrangement = None
        self.solver_graph_vertices = [] # gui
        self.solver_graph_edges = [] # gui

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

    def solve_thread(self):
        """
        The thread that is run by the "solve" function"
        """
        solver = self.solver_class.get_arguments()
        solver.set_verbose(self.writer)
        solver.load_scene(self.discopygal_scene)
        self.paths = solver.solve()
        self.solver_graph = solver.get_graph()
        self.solver_arrangement = solver.get_arrangement()

    def solve(self):
        """
        This method is called by the solve button.
        Run the MP solver in parallel to the app.
        """
        if self.solver_class is None:
            return
        self.disable_toolbar()
        self.worker = Worker(self.solve_thread)
        self.worker.signals.finished.connect(self.solver_done)
        self.threadpool.start(self.worker)

robot_setup = RobotSetup()

class RobotEnvironment:

    def __init__(self,base_length=2, scale=1):
        self.robot_setup = RobotSetup()
        self.base_length = base_length
        self.scale = scale
        self.length = self.base_length*self.scale
        self.points = []
        self.segments = []
        self.path = None
        self._gen_env()


    def _gen_env(self):
        pass
        # self.points.append(PathPoint(0, 0))
        # self.points.append(Point(0, self.length))
        # self.points.append(Point( self.length,  self.length))
        # self.points.append(Point( self.length, 0))
        # self.segments.append(Segment_2(p0, p1))
        # self.segments.append(Segment_2(p1, p2))
        # self.segments.append(Segment_2(p2, p3))
        # self.segments.append(Segment_2(p3, p0))

#env = RobotEnvironment()

import numpy as np
class RobotPath:
    def __init__(self, env):
        self.points = []
        self.rays = []
        self.path_for_robot = []
        #self._gen_path(env)
        self._gen_arc_path_sin(env)

    def _calc_angle_rad(self, p1:Point_2, p2:Point_2):
        dx = p2.x().to_double() - p1.x().to_double()
        dy = p2.y().to_double() - p1.y().to_double()
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
        self.points = [Point_2(x, math.sin(3*x)/2+1) for x in X]
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


if __name__ == '__main__':
    # Setup the scene
