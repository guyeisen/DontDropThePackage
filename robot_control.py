import time
import math
from typing import List, Any

import numpy as np
from discopygal.bindings import Ker

from path_optimizations import PathSection
from robomaster import robot
from robomaster import version
from robomaster import led

# robomaster.config.LOCAL_IP_STR = "172.17.230.195"  # needed only when there are multiple network cards
# robomaster.config.DEFAULT_CONN_TYPE = "sta"
# robomaster.config.DEFAULT_PROTO_TYPE = "tcp"
# ep_robot.initialize()

# ---------- move_straight: ---------------

ROBOT_WIDTH = 0.240
ROBOT_LENGTH = 0.295
WHEEL_WITDH = 0.04
WHEEL_RADIUS = 0.095/2

a = ROBOT_WIDTH / 2 - WHEEL_WITDH / 2
b = ROBOT_LENGTH / 2 - WHEEL_RADIUS

CONST = (a*math.sin(math.radians(45)) + b*math.cos(math.radians(45)))/ math.sin(math.radians(45))

class RobotControl:
    def __init__(self):
        self.ep_robot = robot.Robot()
        # sanity check:
        sdk_version = version.__version__
        print("sdk version:", sdk_version)
        # init:
        # conn_type: 'ap'=WiFi direct, 'sta'=networking. proto_type:'tcp'/'udp'
        self.ep_robot.initialize(conn_type='ap',proto_type='tcp')
        # ------------------------- Obtain module objects: ----------------------------
        self.ep_chassis = self.ep_robot.chassis
        self.ep_led = self.ep_robot.led

        # ----------- set led to purple: ----------
        self.ep_led.set_led(comp=led.COMP_ALL, r=60, g=0, b=60, effect=led.EFFECT_ON)

    def run_path(self,path: List[PathSection]):
        """runs the given path"""
        speed= 0.3
        for section in path:
            if section.is_first_movement:
                self.rotate(section.angle_end)
            if section.isCircle:

                self.move_circle_Husband(speed=section.speed_end,
                                         R=section.radius,
                                         theta=section.arc_angle,
                                         should_stop=section.should_stop,
                                         circle_orient=section.KerElement.orientation())

            if not section.isCircle and section.distance != 0:# means this is straight segment
                if section.isGlide:

                    self.glide_smoothly(start_speed=section.speed_start,
                                        end_speed=section.speed_middle,
                                        distance=section.part1_dis,
                                        should_stop=section.should_stop)
                    self.glide_smoothly(start_speed=section.speed_middle,
                                        end_speed=section.speed_end,
                                        distance=section.part2_dis,
                                        should_stop=section.should_stop)
                else:
                    self.move_straight_exact(distance=section.distance,speed=section.speed_end)
        self.stop()


    def drive_rpm(self, w1, w2, w3, w4, timeout, should_stop=True,prevent_stop_factor=0.9):
        #print(f"rpm: {np.average([w1,w2,w3,w4])} for {timeout}s")
        self.ep_chassis.drive_wheels(w1=w1, w2=w2, w3=w3, w4=w4, timeout=timeout)
        if should_stop:
            time.sleep(timeout)
        else:
            time.sleep(prevent_stop_factor*timeout)

    # def get_time_for_glide(self, start_speed, end_speed, distance, intervals=20):
    #     """calculate and returns the total time for glide through speeds
    #         THIS DOES NOT PERFORM ANY DRIVE ACTION"""
    #     time_for_action = 0
    #     S = np.linspace(start=start_speed ** 2, stop=end_speed ** 2, num=int(intervals))
    #     speeds = [math.sqrt(s) for s in S]
    #     for i, s in enumerate(speeds):
    #         t = distance / (intervals * s)
    #         time_for_action += t
    #     return time_for_action

    def get_intervals_num(self, distance, start_speed, end_speed, max_speed_jump=0.01, max_interval_len=0.03):
        intervals_num = int(math.ceil(math.fabs(end_speed - start_speed) / max_speed_jump) + 1)
        if distance / intervals_num < max_interval_len:
            intervals_num = int(distance // max_interval_len)
        if start_speed != end_speed and intervals_num < 2:
            intervals_num = 2
        return intervals_num


    def glide_smoothly(self, start_speed, end_speed, distance,  func:Any = lambda x:x, oposite_func:Any = lambda x:x ,should_stop=False, proportion=0.8):
        """
        glide through start_speed (m/s) to end_speed (m/s) whithin distance(m)
        intervals are the number of speed changes the robot will make
        func and opposite_func are used in order to make the accelaration more general.
        we first us func on both start_speed and end_speed, then section it to equal parts, and the lowring it back to the original speeds
        with opposite_func.
        for example func can be math.exp and opposite_func will be math.log
        Different functions result in different movement and accelaraion.
        MOVES ONLY IN STRAIGHT LINE.
        feature: WILL NOT stop at the end of the run. possibly will need to change this.
        """

        intervals = self.get_intervals_num(distance, start_speed, end_speed)

        if start_speed==end_speed:
            speeds = [start_speed for i in range(intervals)]
        else:
            S = np.linspace(start=func(start_speed), stop=func(end_speed), num=int(intervals))

            speeds = [oposite_func(s) for s in S]

        proportion=1
        for i, s in enumerate(speeds):
            if s==0:
                continue
            rpm = speed_to_rpm(s)
            t = distance*proportion/(intervals*s)
            if i == len(speeds) - 1:
                self.drive_rpm(w1=rpm, w2=rpm, w3=rpm, w4=rpm,timeout=t,prevent_stop_factor=1)
            else:
                self.drive_rpm(w1=rpm, w2=rpm, w3=rpm, w4=rpm,timeout=t,prevent_stop_factor=1)



    def rotate(self,theta):
        """rotates the robot at a hardcoded slow speed in angle theta (degrees)"""
        t_theta_s100 = 2.375 * (theta / 360) * 3
        sign = 1 if theta >= 0 else -1
        self.ep_chassis.drive_wheels(w1=sign * 100/3, w2=sign * (-100)/3, w3=sign * (-100)/3, w4=sign * (100)/3,
                                     timeout=sign * t_theta_s100)  # 180 deg
        time.sleep(sign * t_theta_s100)


    def move_straight_exact(self, distance, speed=0.5):
        """distance is in meters
            speed in m/s """
        direction = 1 if distance >= 0 else -1
        t = math.fabs(distance / speed)
        rpm = direction * speed_to_rpm(speed)
        print(f"running for {t} sec. rpm is {rpm}")
        self.ep_chassis.drive_wheels(w1=rpm, w2=rpm, w3=rpm, w4=rpm, timeout=t)
        time.sleep(t)


    def rotate_and_go(self, theta, distance, speed=0.3):
        t_theta_s100 = 2.36 * (theta / 360)
        sign = 1 if theta >= 0 else -1
        self.ep_chassis.drive_wheels(w1=sign * 100, w2=sign * (-100), w3=sign * (-100), w4=sign * (100),
                                timeout=sign * t_theta_s100)  # 180 deg
        time.sleep(sign * t_theta_s100)
        self.move_straight_exact(distance, speed)


    def get_length_cos_theorm(self,a, b, gamma):
        return math.sqrt(a**2 + b**2 - 2*a*b*math.cos(gamma))

    def get_speeds_by_radiuses(self, r_out, r_in, speed, factor):
        vout = 2 * r_out * speed * factor / (r_out * factor + r_in)
        vin = 2 * speed - vout
        return vout, vin

    def factor(self,C, alpha1, beta1):
        return beta1*((C+1)*alpha1 + (C-1)*beta1)/((C-1)*alpha1 + (C+1)*beta1)

    def factor2(self, C, alpha1, beta1, wanted_speed):
        return wanted_speed*(((C+1)*alpha1 + (C-1)*beta1)/(C*(alpha1+beta1)))



    def calc_wanted_rpms(self, rpm, R):
        outer = (rpm*(CONST+R))/R
        inner = 2*rpm - outer
        return outer, inner

    def move_circle_Husband(self, speed=0.3, R=0.6, theta=math.pi/2, should_stop=False, circle_orient=None):
        """
        moves in the given radius R in the given speed (m/s)
        """
        wanted_rpm = speed_to_rpm(speed)
        rpm_out, rpm_in = self.calc_wanted_rpms(wanted_rpm, R)
        wanted_omega = speed / R
        if theta < -math.pi:
            theta = 2*math.pi + theta
        time_for_theta = math.fabs(theta/ (wanted_omega))
        if circle_orient == Ker.CLOCKWISE:
            self.drive_rpm(w1=rpm_in, w2=rpm_out, w3=rpm_out, w4=rpm_in, timeout=time_for_theta)
        elif circle_orient == Ker.COUNTERCLOCKWISE:
            self.drive_rpm(w1=rpm_out, w2=rpm_in, w3=rpm_in, w4=rpm_out, timeout=time_for_theta)
        if should_stop:
            self.stop()


    def stop(self):
        self.ep_chassis.drive_wheels(0, 0, 0, 0)
        time.sleep(0.5)


def speed_to_rpm(speed):
    """meters per second to rpm"""
    return 60*speed/(2*math.pi*WHEEL_RADIUS)

def rpm_to_speed(rpm):
    return 2*math.pi*WHEEL_RADIUS*rpm/60

#