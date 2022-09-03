import math
import os
import importlib.util
import sys
import json
import importlib.util
import time
import traceback
from inspect import isclass
from PyQt5 import QtWidgets
# from CGALPY.Ker import Ray_2, Direction_2
from collision_detection import ObjectCollisionDetection
from path_optimizations import parse_path2
from robot_control import *
from discopygal.bindings import Point_2
from discopygal.bindings import *


from discopygal.solvers import SceneDrawer
from discopygal.solvers import Scene
from discopygal.solvers import Solver

from smooth_path import get_angle_of_point
from solver_viewer_main import SolverViewerGUI

DEFAULT_SCENE = "small_scene.json"


class EnviromentConfigurations:
    def __init__(self):
        self.app = QtWidgets.QApplication(sys.argv)
        self.gui = SolverViewerGUI()
        self.gui.solve()
        # self.gui.toggle_paths()

        #self.mygui = SolverViewerGUI()
        # self.path_edges = []
        # self.path_vertices = []
        # self.writer = None
        # self.discopygal_scene = Scene()
        # self.scene_path = ""

        # self.paths = None
        # # Setup solver
        # self.solver_class = None
        # #self.load_solver()
        # self.solver_graph = None
        # self.solver_arrangement = None
        # self.solver_graph_vertices = [] # gui
        # self.solver_graph_edges = [] # gui
        #self.execute_defaults()



    def execute_defaults(self):
        self.gui.mainWindow.show()
        self.load_scene()
        self.solver_from_file()
        self.gui.solve()#############was without gui,
        self.gui.toggle_paths()


    def load_scene(self):
        """
        Load default scene.
        """
        with open(DEFAULT_SCENE, 'r') as fp:
            d = json.load(fp)
        self.gui.discopygal_scene = Scene.from_dict(d)
        scene_drawer = SceneDrawer(self.gui, self.gui.discopygal_scene)
        scene_drawer.draw_scene()
        self.clear_paths()


    def clear_paths(self):
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

    # def solve_thread(self):
    #     """
    #     The thread that is run by the "solve" function"
    #     """
    #     solver_args = self.get_solver_args()
    #     #solver.set_verbose(self.writer)
    #     solver = self.solver_class.from_arguments(solver_args)
    #     solver.load_scene(self.gui.discopygal_scene)
    #     self.gui.paths = solver.solve()
    #     self.gui.solver_graph = solver.get_graph()
    #     self.gui.solver_arrangement = solver.get_arrangement()

    # def solver_from_file(self):
    #     """
    #     Loads the solver
    #     """
    #     path = "prm.py"
    #     try:
    #         spec = importlib.util.spec_from_file_location(path, path)
    #         module = importlib.util.module_from_spec(spec)
    #         spec.loader.exec_module(module)
    #         cnt = 0
    #         for obj_name in dir(module):
    #             obj = getattr(module, obj_name)
    #             if isclass(obj) and issubclass(obj, Solver) and obj_name != "Solver":
    #                 self.solver_class = obj
    #                 globals()[obj_name] = obj
    #                 cnt += 1
    #     except Exception as e:
    #         print("Could not import module", repr(e))


    # def solve(self):
    #     """
    #     This method is called by the solve button.
    #     Run the MP solver in parallel to the app.
    #     """
    #     if self.solver_class is None:
    #         print("self.solver_class is None")
    #         return
    #     self.solve_thread()





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
        print(f"Calculated {len(self.points)} points")
        self.rays = [Ker.Ray_2(self.points[i].location, self._get_Direction_2(self.points[i].location, self.points[i + 1].location)) for i in range(len(self.points) - 1)]
        return parse_path(self.rays, self.points[len(self.points)-1].location)

def _calc_angle_rad_fromDirection(direction: Direction_2):
    return math.atan2(direction.dy().to_double(), direction.dx().to_double())


def to_degrees(rad):
    return 180 * rad / math.pi


def parse_path(rays: [Ker.Ray_2], last_point: Point_2):
    """
    returns list of tuples: (starting angle, distance)
                    (p1)
                    /  \
    prev_distance  /    \ curr_distance
                  /      \
               (p0)      (p2)
    """
    angle_distance_list = []
    p0 = np.zeros(2)
    p1 = np.zeros(2)
    p2 = np.zeros(2)
    print(f"Calculated {len(rays)} rays")
    for i in range(1, len(rays)-1):
        p0[0] = rays[i-1].source().x().to_double()
        p0[1]= rays[i-1].source().y().to_double()
        p1[0] = rays[i].source().x().to_double()
        p1[1]= rays[i].source().y().to_double()
        p2[0] = rays[i + 1].source().x().to_double()
        p2[1] = rays[i + 1].source().y().to_double()
        #if p2[0]
        prev_distance = np.linalg.norm(p1-p0)
        curr_distance = np.linalg.norm(p2-p1)
        #time_for_seg = time_per_length(distance)
        angle1 = _calc_angle_rad_fromDirection(rays[i].direction())
        angle0 = _calc_angle_rad_fromDirection(rays[i - 1].direction())
        angle = min(angle1 - angle0, np.pi - (angle1 - angle0))
        if i == 1:#first iteration
            print(f'First Angle: {to_degrees(angle0)}, Distance: {prev_distance}')
            angle_distance_list.append((to_degrees(angle0),prev_distance))
        print(f'Angle: {to_degrees(angle)}, Distance: {curr_distance}')

        angle_distance_list.append((to_degrees(angle), curr_distance))
        if i==len(rays)-2:
            last_p = np.zeros(2)
            last_p[0] = last_point.x().to_double()
            last_p[1] = last_point.y().to_double()
            distance = np.linalg.norm(last_p-p2)
            angle2 = _calc_angle_rad_fromDirection(rays[i+1].direction())
            angle = min(angle2 - angle1, np.pi - (angle2 - angle1))
            print(f'Last Angle: {to_degrees(angle)}, Distance: {distance}')
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

def slalum(control: RobotControl, rpm):
    control.glide_smoothly(start_speed=0.0, end_speed=rpm, distance=0.6)
    control.move_circle_Husband(speed=rpm, R=0.3, theta=math.pi, circle_orient=Ker.CLOCKWISE)
    control.move_straight_exact(distance=0.6, speed=rpm)
    control.move_circle_Husband(speed=rpm, R=0.3, theta=math.pi, circle_orient=Ker.COUNTERCLOCKWISE)
    control.move_straight_exact(distance=0.6, speed=rpm)
    control.move_circle_Husband(speed=rpm, R=0.3, theta=math.pi, circle_orient=Ker.CLOCKWISE)
    control.glide_smoothly(start_speed=rpm, end_speed=0.0, distance=0.6)


def get_robot():
    control = None
    while control == None:
        try:
            control = RobotControl()
        except Exception as e:
            print("GOT ERROR CONNECTING, TRYING AGAIN")
    print("ROBOT CONNECTED")
    #time.sleep(10)
    return control

def testings(control):
    control.glide_smoothly(start_speed=0.0, end_speed=1.5, distance=5,func=lambda x:x, oposite_func=lambda x:x)
    control.glide_smoothly(start_speed=0.0, end_speed=1.5, distance=5, func=math.exp, oposite_func=math.log)
    control.move_circle_BF(speed=0.7, R = 0.6, theta=4*math.pi)
    control.move_straight_exact(distance=2,speed=0.6)
    control.begin_slowly_to_speed(speed=0.6)
    control.move_straight_exact(distance=0.6, speed=0.5)

    slalum(control, rpm=0.3)

    control.move_circ_2(R=1,speed=0.5,alpha=0)
    control.begin_slowly_to_speed(0.3)
    control.ep_chassis.drive_wheels(w1=60, w2=60, w3=60, w4=60, timeout=5)
    time.sleep(5)

    # ------------- move robot: ---------
    # robot_path = RobotPath(env.gui.paths.paths[robot].points)
    # run_path(control, robot_path.path_for_robot)
def draw_paths(env, optimize=False):
    while (env.gui.paths_optimized == None):
        # print(f"Waiting for path to be created...")
        pass
    print("Path created!")
    env.gui.toggle_paths(optimize)

def end_robot(control):
    # ----------- set led to grey: ----------#
    control.ep_led.set_led(comp=led.COMP_ALL, r=10, g=10, b=10, effect=led.EFFECT_ON)
    control.ep_robot.close()

def replace_path_with_my_path(path): # todo temp delete
    p0 = Point_2(-1, 2)
    p1 = Point_2(0, 0)
    p2 = Point_2(2, 4)
    p3 = Point_2(5, -2)
    p4 = Point_2(3, -6)
    p5 = Point_2(4, -8)
    my_points = [p0, p1, p2, p3, p4, p5]
    for i in range(len(path.points)):
        path_point = path.points[i]
        path_point.location = my_points[i]

def replace_path_for_simple_scene(env): # todo temp delete
    # --- temp - path for "simple_scene": ---
    p0 = Point_2(-4, -4)
    p1 = Point_2(3, -2) # Point_2(2.97521, -1.92828)
    p2 = Point_2(-4.5, 1.5) # Point_2(-4.30751, 1.47929)
    p3 = Point_2(3, 2)
    my_points = [p0, p1, p2, p3]
    robot = env.gui.discopygal_scene.robots[0]
    path = None
    while path is None:
        path = env.gui.paths_optimized.paths.get(robot)
    for i in range(4):
        path_point = path.points[i]
        path_point.location = my_points[i]
    path.points = path.points[:4]

def print_smooth_path(smooth_path):
    print("\nsmooth path: ")
    for i in range(len(smooth_path)):
        if type(smooth_path[i]) is Ker.Circle_2:
            c = smooth_path[i]
            r = math.sqrt(c.squared_radius().to_double())
            prev_seg = smooth_path[i-1]
            next_seg = smooth_path[i+1]
            arc_source_angle = get_angle_of_point(c, prev_seg.target())
            arc_target_angle = get_angle_of_point(c, next_seg.source())
            print(f"circle_{i} -- center: {c.center()} -- radius: {r} -- source_point: {prev_seg.target()} -- target_point: {next_seg.source()}"
                  f" \n-- start_angle: {np.rad2deg(arc_source_angle)} -- end_angle: {np.rad2deg(arc_target_angle)} -- orient: {c.orientation()}\n")
        else:
            seg = smooth_path[i]
            print(f"seg_{i} -- {seg}\n")

def robot_tests(control):
    speed = 0.9
    end_speed = 0.0
    R = 2
    control.glide_smoothly(start_speed=0.0, end_speed=speed, distance=1, func=lambda x: x, oposite_func=lambda x: x)
    # control.glide_smoothly(start_speed=0.0, end_speed=1.5, distance=5, func=math.exp, oposite_func=math.log)
    control.move_circle_Husband(speed=speed, R=R, theta=math.pi,circle_orient=Ker.CLOCKWISE,should_stop=False)
    control.glide_smoothly(start_speed=speed, end_speed=end_speed, distance=0.2, func=lambda x: x, oposite_func=lambda x: x,intervals=7)
    #control.move_straight_exact(distance=1, speed=end_speed)
    # control.move_straight_exact(distance=2, speed=0.6)
    # control.begin_slowly_to_speed(speed=0.6)
    # control.move_straight_exact(distance=0.6, speed=0.5)
    end_robot(control)
# -------------------------------------------- main: -------------------------------------------------------
def finished(path):
    print("IM BACK TO MAIN")
    control: RobotControl = get_robot()
    robot_tests(control)

if __name__ == '__main__':
    try:
        #control:RobotControl = get_robot()
        #robot_tests(control)
        #testings(control)
        env = EnviromentConfigurations()
        #env.gui.worker.signals.finished.connect()
        print("FINISH")
        #draw_paths(env,optimize=True)

        # while env.gui.smooth_path is None:
        #     print("Waiting for path creation...")
        #     pass

        # path_for_robot = parse_path2(env.gui.smooth_path)
        # control.run_path(path_for_robot)

        # replace_path_with_my_path(path) # todo temp delete
        # replace_path_for_simple_scene(env) # todo temp delete



        # print_smooth_path() # todo temp delete

        env.gui.mainWindow.show()

        # end_robot(control)
    except Exception as e:
        print(repr(e))
        traceback.print_exc()

    sys.exit(env.app.exec_())