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

from smooth_path import get_angle_of_point
from solver_viewer_main import SolverViewerGUI

DEFAULT_SCENE = "small_scene.json"


class EnviromentConfigurations:
    def __init__(self):
        self.app = QtWidgets.QApplication(sys.argv)
        self.gui = SolverViewerGUI()
        self.gui.solve()



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

    def gen_robot_path_from_points(self):#LEGACY DONT USE
        print(f"Calculated {len(self.points)} points")
        self.rays = [Ker.Ray_2(self.points[i].location, self._get_Direction_2(self.points[i].location, self.points[i + 1].location)) for i in range(len(self.points) - 1)]
        return parse_path(self.rays, self.points[len(self.points)-1].location)

def _calc_angle_rad_fromDirection(direction: Direction_2):
    return math.atan2(direction.dy().to_double(), direction.dx().to_double())


def to_degrees(rad):
    return 180 * rad / math.pi


def parse_path(rays: [Ker.Ray_2], last_point: Point_2):#LEGACY DONT USE
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
        prev_distance = np.linalg.norm(p1-p0)
        curr_distance = np.linalg.norm(p2-p1)
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


def run_path(control: RobotControl, path): #LEGACY DONT USE
    for angle, distance in path:
        print(f'Angle: {angle}, Distance: {distance}')
        control.rotate_and_go(angle, distance)

def slalum(control: RobotControl, speed):
    control.glide_smoothly(start_speed=0.0, end_speed=speed, distance=0.6)
    control.move_circle_Husband(speed=speed, R=0.3, theta=math.pi, circle_orient=Ker.CLOCKWISE)
    control.move_straight_exact(distance=0.6, speed=speed)
    control.move_circle_Husband(speed=speed, R=0.3, theta=math.pi, circle_orient=Ker.COUNTERCLOCKWISE)
    control.move_straight_exact(distance=0.6, speed=speed)
    control.move_circle_Husband(speed=speed, R=0.3, theta=math.pi, circle_orient=Ker.CLOCKWISE)
    control.glide_smoothly(start_speed=speed, end_speed=0.0, distance=0.6)


def get_robot():
    control = None
    while control == None:
        try:
            control = RobotControl()
        except Exception as e:
            print("GOT ERROR CONNECTING, TRYING AGAIN")
    print("ROBOT CONNECTED")
    return control

def testings(control):
    control.glide_smoothly(start_speed=0.0, end_speed=1.5, distance=5,func=lambda x:x, oposite_func=lambda x:x)
    control.glide_smoothly(start_speed=0.0, end_speed=1.5, distance=5, func=math.exp, oposite_func=math.log)
    control.move_circle_BF(speed=0.7, R = 0.6, theta=4*math.pi)
    control.move_straight_exact(distance=2,speed=0.6)
    control.begin_slowly_to_speed(speed=0.6)
    control.move_straight_exact(distance=0.6, speed=0.5)
    slalum(control, speed=0.3)
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
    print("finished path search")
    env.gui.toggle_paths(optimize)

def end_robot(control):
    # ----------- set led to grey: ----------#
    control.ep_led.set_led(comp=led.COMP_ALL, r=10, g=10, b=10, effect=led.EFFECT_ON)
    control.ep_robot.close()
def robot_path_not_found(control):
    # ----------- set led to grey: ----------#
    control.ep_led.set_led(comp=led.COMP_ALL, r=10, g=0, b=0, effect=led.EFFECT_ON)
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
    control.glide_smoothly(start_speed=0.0, end_speed=0.5, distance=3.6, func=lambda x: x, oposite_func=lambda x: x)
    control.stop()
    end_robot(control)


def finished_solving(path_created=True, writer=None):
    if writer is not None:
        print("Hurray I'm ready to run! Click Play!",file=writer)


def parse_and_run(smooth_path):
    control: RobotControl = get_robot()
    min_linear_deceleration = -0.5
    max_linear_acceleration = 0.5
    max_centripetal_acceleration = 0.2

    path_for_robot = parse_path2(smooth_path, max_linear_acceleration, min_linear_deceleration,
                                 max_centripetal_acceleration)

    control.run_path(path_for_robot)
    end_robot(control)

# -------------------------------------------- main: -------------------------------------------------------

if __name__ == '__main__':
    try:
        env = EnviromentConfigurations()

        env.gui.mainWindow.show()

    except Exception as e:
        print(repr(e))
        traceback.print_exc()

    sys.exit(env.app.exec_())