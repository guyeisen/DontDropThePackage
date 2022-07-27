import time
import math
import numpy as np
from robomaster import robot
from robomaster import version
from robomaster import led

# robomaster.config.LOCAL_IP_STR = "172.17.230.195"  # needed only when there are multiple network cards
# robomaster.config.DEFAULT_CONN_TYPE = "sta"
# robomaster.config.DEFAULT_PROTO_TYPE = "tcp"
# ep_robot.initialize()

# ---------- move_straight: ---------------
def move_straight(distance, speed=1):
    """
    distance is in cm.
    """
    scaled_distance = distance/60
    # 60 m = 1.5 timeout
    sign = 1 if distance >= 0 else -1
    wheelspeed = 70*speed*sign
    timeout = 1.5 * scaled_distance * sign
    res = ep_chassis.drive_wheels(wheelspeed, wheelspeed, wheelspeed, wheelspeed, timeout)
    time.sleep(timeout)
    return res

# ----------- move_half_meter_in_angle_theta: ----------------------------
def move_half_meter_in_angle_theta(theta):
    rad_theta = np.deg2rad(theta)
    w1 = w3 = 70 * (math.sin(rad_theta) + math.cos(rad_theta))
    w2 = w4 = 70 * (math.sin(rad_theta) - math.cos(rad_theta))
    res = ep_chassis.drive_wheels(w1, w2, w3, w4, timeout=2.5)
    time.sleep(1)
    return res

def big_circle_theta_deg(theta):
    timeout = 9.7 * (theta/360)
    ep_chassis.drive_wheels(w1=50, w2=100, w3=100, w4=50, timeout=timeout)
    time.sleep(timeout)


def stop():
    ep_chassis.drive_wheels(0,0,0,0)
    time.sleep(0.5)

if __name__ == '__main__':
    ep_robot = robot.Robot()

    # sanity check:
    sdk_version = version.__version__
    print("sdk version:", sdk_version)

    # init:
    ep_robot.initialize(conn_type='ap',
                        proto_type='tcp')  # conn_type: 'ap'=WiFi direct, 'sta'=networking. proto_type:'tcp'/'udp'

    # ------------------------- Obtain module objects: ----------------------------
    ep_chassis = ep_robot.chassis
    ep_led = ep_robot.led

    # ----------- set led to purple: ----------
    ep_led.set_led(comp=led.COMP_ALL, r=60, g=0, b=60, effect=led.EFFECT_ON)


    # move_half_meter_in_angle_theta(30)

    # -- 90 ged small circle --
    # ep_chassis.drive_wheels(w1=-10, w2=100, w3=100, w4=-10, timeout=1.30)
    # time.sleep(1.30)

    # -- 360 ged small circle --
    # ep_chassis.drive_wheels(w1=-10, w2=100, w3=100, w4=-10, timeout=5.2)
    # time.sleep(5.2)

    # -- 90 ged big circle --
    # ep_chassis.drive_wheels(w1=50, w2=100, w3=100, w4=50, timeout=2.50)
    # time.sleep(2.55)

    # -- 180 ged big circle --
    # ep_chassis.drive_wheels(w1=50, w2=100, w3=100, w4=50, timeout=5.00)
    # time.sleep(5.05)

    # -- 360 ged big circle --
    # ep_chassis.drive_wheels(w1=50, w2=100, w3=100, w4=50, timeout=9.70)
    # time.sleep(9.75)

    # big_circle_theta_deg(90)
    # move_straight(30)
    # big_circle_theta_deg(90)

    # big_circle_theta_deg(45)
    # big_circle_theta_deg(45)
    # big_circle_theta_deg(180)

    # -- 90 deg radius of 1.5 "shibers"  25.5 cm--
    # ep_chassis.drive_wheels(w1=0, w2=100, w3=100, w4=0, timeout=1.3)
    # time.sleep(1.3)

    # -- 90 deg radius of 2 "shibers"  38 cm--
    # ep_chassis.drive_wheels(w1=30, w2=100, w3=100, w4=30, timeout=1.75)
    # time.sleep(1.75)

    # -- 90 deg radius of 3.25 "shibers" 65cm --
    # ep_chassis.drive_wheels(w1=50, w2=100, w3=100, w4=50, timeout=2.4)
    # time.sleep(2.4)


    # -- 90 deg radius of 4.5 "shibers" 85 cm --
    # ep_chassis.drive_wheels(w1=60, w2=100, w3=100, w4=60, timeout=3)
    # time.sleep(3)

    # ---- slalom: ---
    move_straight(30)
    ep_chassis.drive_wheels(w1=30, w2=100, w3=100, w4=30, timeout=1.75)
    time.sleep(1.75)
    move_straight(30)
    ep_chassis.drive_wheels(w1=100, w2=30, w3=30, w4=100, timeout=1.75)
    time.sleep(1.75)
    move_straight(30)
    ep_chassis.drive_wheels(w1=30, w2=100, w3=100, w4=30, timeout=1.75)
    time.sleep(1.75)
    move_straight(30)
    stop()

    # ep_chassis.drive_wheels(w1=-10, w2=200, w3=200, w4=-10, timeout=0.6)
    # time.sleep(0.7)
    # ep_chassis.drive_wheels(w1=70, w2=70, w3=70, w4=70, timeout=1.5)
    # time.sleep(2)

    ep_robot.close()
