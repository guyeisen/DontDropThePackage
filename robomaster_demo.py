import time
import math
import numpy as np
import robomaster
from robomaster import robot
from robomaster import version
from robomaster import chassis
from robomaster import gimbal
from robomaster import led
from poc import RobotEnvironment, RobotPath, generate_environment

# sanity check:
sdk_version = version.__version__
print("sdk version:", sdk_version)

ep_robot = robot.Robot()

# robomaster.config.LOCAL_IP_STR = "172.17.230.195"  # needed only when there are multiple network cards
# robomaster.config.DEFAULT_CONN_TYPE = "sta"
# robomaster.config.DEFAULT_PROTO_TYPE = "tcp"
# ep_robot.initialize()

ep_robot.initialize(conn_type='ap',
                    proto_type='tcp')  # conn_type: 'ap'=WiFi direct, 'sta'=networking. proto_type:'tcp'/'udp'

# ------------------------- Obtain module objects: ----------------------------
ep_chassis = ep_robot.chassis
ep_led = ep_robot.led

# ------- move (Chassis relative position control (by distance)): -------
# ep_chassis.move(x=0, y=0, z=0, xy_speed=0.8, z_speed=100).wait_for_completed()
# ep_chassis.move(x=0, y=0.5, z=0, xy_speed=0.8, z_speed=100).wait_for_completed()

# ----------- set led to purple: ----------
ep_led.set_led(comp=led.COMP_ALL, r=60, g=0, b=60, effect=led.EFFECT_ON)


# def move_half_meter_over_an_axis(axis, direction):
#     if axis == 'x':
#         ep_chassis.drive_wheels(w1=direction * (-70), w2=direction * 70, w3=direction * (-70), w4=direction * 70,
#                                 timeout=1)
#         time.sleep(1)
#     elif axis == 'y':
#         s = direction * 70
#         ep_chassis.drive_wheels(w1=s, w2=s, w3=s, w4=s, timeout=1)
#         time.sleep(1)


def move_half_meter_in_angle_theta(theta):
    rad_theta = np.deg2rad(theta)
    w1 = w3 = 70 * (math.sin(rad_theta) + math.cos(rad_theta))
    w2 = w4 = 70 * (math.sin(rad_theta) - math.cos(rad_theta))
    res = ep_chassis.drive_wheels(w1, w2, w3, w4, timeout=2.5)
    time.sleep(1)
    return res

# for i in range(2):
#     move_half_meter_over_an_axis('y', 1)
#     move_half_meter_over_an_axis('x', 1)
#     move_half_meter_over_an_axis('y', -1)
#     move_half_meter_over_an_axis('x', -1)


# ------ Holonomic movement in S shape: ----------------
# for theta in [0, 30, 90, 120, 180, 180, 120, 90, 30, 0]:
#     move_half_meter_in_angle_theta(theta)


# ------------ 90 deg with arc  -----------
# ep_chassis.drive_wheels(w1=-10, w2=100, w3=100, w4=-10, timeout=1.25)
# time.sleep(1.3)
# # ep_chassis.drive_wheels(w1=-10, w2=200, w3=200, w4=-10, timeout=0.6)
# # time.sleep(0.7)
# ep_chassis.drive_wheels(w1=70, w2=70, w3=70, w4=70, timeout=1.5)
# time.sleep(2)

# -------------- rotate: --------------
import numpy as np
def rotate(theta):
    # speed 100 for all wheels, w1 and w4 are negative
    # timeout: 2.36 for 360 deg
    t_theta_s100 = 2.36 * (theta/360)
    sign = 1 if theta>= 0 else -1
    ep_chassis.drive_wheels(w1=sign*100, w2=sign*(-100), w3=sign*(-100), w4=sign*(100), timeout=sign*t_theta_s100) # 180 deg
    time.sleep(sign*t_theta_s100 + 0.1)


def move_straight(distance, speed=1): # dis in meters
    """
    distance is in meters.
    """

    scaled_distance = distance/0.6
    # 0.60 m = 1.5 timeout
    wheelspeed=70*speed
    res = ep_chassis.drive_wheels(wheelspeed, wheelspeed, wheelspeed, wheelspeed, timeout=1.5*scaled_distance)
    #res = ep_chassis.drive_wheels(wheelspeed, wheelspeed, wheelspeed, wheelspeed, timeout=1.5 * scaled_distance / speed)
    #2 * scaled_distance
    time.sleep(2*scaled_distance)
    return res

# --------- get path: ----------
# env = generate_environment()
# robot_path = env.path.path_for_robot


# print([[ray.source.x, ray.source.y, ray.angle] for ray in rays_list])

if __name__ == '__main__':
    rotate(360)


    env = generate_environment()
    robot_path = env.path.path_for_robot
    for tup in robot_path:
        rotate(tup[0])
        print(f'Angle: {tup[0]}, Distance: {tup[1]}')
        move_straight(tup[1])
    #move_straight(0.3)


ep_robot.close()