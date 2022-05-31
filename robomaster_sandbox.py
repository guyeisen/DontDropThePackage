import time
import robomaster
from robomaster import robot
from robomaster import version
from robomaster import chassis
from robomaster import gimbal
from robomaster import led

# https://robomaster-dev.readthedocs.io/en/latest/python_sdk/beginner_ep.html?highlight=set_robot_mode

# --------- Establish a Wi-Fi connection: ---------
# https://robomaster-dev.readthedocs.io/en/latest/text_sdk/connection.html
# https://robomaster-dev.readthedocs.io/en/latest/python_sdk/connection.html
# Open the wireless network access list on the PC, select the Wi-Fi hotspot name displayed on the sticker on the robot body,
# enter the 8-digit password, and then select “Connect”.

# sanity check:
sdk_version = version.__version__
print("sdk version:", sdk_version)

# Configs:
# robomaster.config.LOCAL_IP_STR = "172.17.230.195"  # needed only when there are multiple network cards
# robomaster.config.DEFAULT_CONN_TYPE = "sta"
# robomaster.config.DEFAULT_PROTO_TYPE = "tcp"


# In Wi-Fi direct connection mode, the default IP address of the robot is 192.168.2.1 and the control command port is port ``40923

''' -------- Robot movement modes: ----------------
The robot movement mode describes the relationship and mutual movement between the gimbal and the chassis. 
Each robot mode corresponds to a specific interaction relationship.
There are three robot movement modes:
1. Gimbal follows chassis: In this mode, the yaw axis of the gimbal always follows the movement of the yaw axis of the chassis. In addition, the gimbal does not respond to the yaw axis control part of all control commands. The affected commands are gimbal movement speed control, gimbal relative position control, and gimbal absolute position control.
2. Chassis follows gimbal: In this mode, the yaw axis of the chassis always follows the movement of the yaw axis of the gimbal. In addition, the chassis does not respond to the yaw axis control part of all control commands. The affected commands are chassis movement speed control, chassis wheel speed control, and chassis relative position control.
3. Free: In this mode, the movement of the yaw axis of the gimbal does not affect the movement of the yaw axis of the chassis, and vice versa.'''
# https://robomaster-dev.readthedocs.io/en/latest/text_sdk/protocol_api.html

# --------------------------------------------------------------------------------------------------------------------
# --------------------------------------------- Robot functions: -----------------------------------------------------
# --------------------------------------------------------------------------------------------------------------------

ep_robot = robot.Robot()

# ------ Get the robot working mode -----------
'''Returns:	Free mode returns free; chassis follows gimbal mode returns gimbal_lead; gimbal follows chassis mode returns chassis_lead'''
ep_robot.get_robot_mode()


ep_robot.get_battery_percentage()  #(???)

# -------------- Initialization: -------------
'''
Parameters:	
conn_type – connection establishment type: ap means using hotspot direct connection; sta means using networking connection, rndis means using USB connection
proto_type – communication method: tcp, udp'''
ep_robot.initialize(conn_type='ap', proto_type='udp') # conn_type: 'ap'=WiFi direct, 'sta'=networking. proto_type:'tcp'/'udp'


# ------------------------- Obtain module objects: ----------------------------
ep_chassis = ep_robot.chassis
ep_led = ep_robot.led
ep_gimbal = ep_robot.gimbal
ep_servo = ep_robot.servo

# -------------------------------------------------------------------------------------------------------------------
# ------------------------------------------ Chassis Controls: ------------------------------------------------------
# -------------------------------------------------------------------------------------------------------------------

# -------drive_speed ---------
'''x – float:[-3.5,3.5], the x-axis movement speed is the forward speed, in m/s
y – float:[-3.5,3.5], the y-axis movement speed is the traverse speed, in m/s
z – float:[-600,600], z axis speed is the rotation speed, unit °/s
timeout – float:(0,inf), if the speed command of the wheat wheel is not received within the specified time, the robot will be actively controlled to stop, the unit is s'''
ep_chassis.drive_speed(x=0.5, y=0, z=0, timeout=5)
time.sleep(3)
ep_chassis.drive_speed(x=0, y=0, z=0, timeout=5)

# ------- drive_wheels ------
'''the forward rotation in the direction of the front of the car is the positive direction, the unit is rpm
w1 – int:[-1000,1000], the speed of the right front wheat wheel
w2 – int:[-1000,1000], the speed of the left front wheat wheel
w3 – int:[-1000,1000], the speed of the left rear wheat wheel
w4 – int:[-1000,1000], the speed of the right rear wheat wheel
timeout – float:(0,inf), if the speed command of the wheat wheel is not received within the specified time, the robot will be actively controlled to stop, the unit is s'''
ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0, timeout=None)

# ------- move (Chassis relative position control (by distance)): -------
'''x – float: [-5,5], x-axis movement distance, in m
y – float: [-5,5], the movement distance of the y-axis, in m
z – float: [-1800,1800], z-axis rotation angle, unit °
xy_speed – float: [0.5,2], xy axis movement speed, unit m/s
z_speed – float: [10,540], z-axis rotation speed, unit °/s
return action object'''
ep_chassis.move(x=0, y=0, z=0, xy_speed=0.5, z_speed=30)

# ----------------------'wait_for_completed' examples: -----------------------------
# 1:
ep_chassis.move(x=0.5, y=0, z=0, xy_speed=0.7).wait_for_completed()
ep_led.set_led(comp=led.COMP_ALL, r=255, g=0, b=0, effect=led.EFFECT_ON)
# 2:
chassis_action = ep_chassis.move(x=0.5, y=0, z=0, xy_speed=0.7)
ep_led.set_led(comp=led.COMP_ALL, r=255, g=0, b=0, effect=led.EFFECT_ON)
chassis_action.wait_for_completed()  # if The action is completed within the specified time, returns True


# ---------------- Subscribe to Chassis Location Information: -------------------
'''Parameters:
cs – int: [0,1] Set the coordinate system of the chassis position, 0 is the current position of the robot, 1 is the power-on position of the robot
freq – enum: (1, 5, 10, 20, 50) Set the push frequency of data subscription data, in Hz
callback –
Callback function, return data (x, y, z):

x:	Distance along the x-axis, in m
y:	Distance in the y-axis direction, in m
z:	Rotation angle in the z-axis direction, unit °
args – variadic arguments
kw – keyword argument
Returns:
bool: data subscription result'''

'''Return values  (???)
<x> <y> <z>: x-axis position (m), y-axis position (m), and yaw angle (°)
Example:
IN: chassis position ?;: Obtain position information of the chassis.
OUT: 1 1.5 20;: Compared with the position of the chassis when the vehicle was turned on,
 the current position of the chassis is 1 m along the x-axis and 1.5 m along the y-axis, with a rotation angle of 20°.'''

# ep_chassis.sub_position(cs=0, freq=5, callback=None, *args, **kw)
ep_chassis.sub_position(cs=0, freq=5, callback=None)

# Unsubscribe from chassis location information:
'''Returns:	bool: the result of canceling the data subscription'''
ep_chassis.unsub_position()

# -------------------- Subscribe to Chassis Acceleration Information: ---------------------
'''Parameters:
freq – enum:(1, 5, 10, 20, 50) Set the push frequency of data subscription data, in Hz
callback –
Callback function, return data (vgx, vgy, vgz, vbx, vby, vbz):
vgx:	The speed in the x direction in the world coordinate system at the time of power-on
or:	The speed in the y direction in the world coordinate system at the time of power-on
fgz:	The z-direction speed in the world coordinate system at the time of power-on
vbx:	The x-direction velocity in the body coordinate system at the current moment
vby:	The velocity in the y direction in the body coordinate system at the current moment
vbz:	The z-direction velocity in the body coordinate system at the current moment
args – variadic arguments
kw – keyword argument
Returns:
bool: data subscription result'''
# ep_chassis.sub_velocity(freq=5, callback=None, *args, **kw)
ep_chassis.sub_velocity(freq=5, callback=None)

# --- Unsubscribe from chassis acceleration information: ---
'''Returns:	bool: the result of canceling the data subscription'''
ep_chassis.unsub_velocity()


# ------------------------------------------------------------------------------------------------------------
# ---------------------------------- Gimbal Control examples: -----------------------------------------------
# ------------------------------------------------------------------------------------------------------------

# ----------- rotate at a certain speed ---------------
'''Parameters:
pitch_speed – float: [-360, 360], pitch axis speed, unit °/s
yaw_speed – float: [-360, 360], yaw axis speed, unit °/s
Returns:
bool: call resul'''
ep_gimbal.drive_speed(pitch_speed=30.0, yaw_speed=30.0)

# -------- move to the specified position -----------------
# the origin of the coordinate axis is the current position
'''Parameters:
pitch – float: [-55, 55], pitch axis angle, unit °
yaw – float: [-55, 55], yaw axis angle, unit °
pitch_speed – float: [0, 540], the speed of the pitch axis, in °/s
yaw_speed – float: [0, 540], yaw axis movement speed, unit °/s
Returns:
return action object'''
ep_gimbal.move(pitch=0, yaw=0, pitch_speed=30, yaw_speed=30)

# ------- move to the specified position -------------------
# the origin of the coordinate axis is the power-on position (The origin of the coordinate plane is the startup position)
'''Parameters:
pitch – int: [-25, 30], pitch axis angle, unit °
yaw – int: [-250, 250], yaw axis angle, unit °
pitch_speed – int: [0, 540], pitch axis motion speed, unit °/s
yaw_speed – int: [0, 540], yaw axis movement speed, unit °/s
Returns:
return action object'''
ep_gimbal.moveto(pitch=0, yaw=0, pitch_speed=30, yaw_speed=30)

# ------------ Control the PTZ back to center ----------------
'''Parameters:
pitch_speed – float: [-360, 360], pitch axis speed, unit °/s
yaw_speed – float: [-360, 360], yaw axis speed, unit °/s
Returns:
return action object'''
ep_gimbal.recenter(pitch_speed=60, yaw_speed=60)

# ---------- Control the gimbal to resume from hibernation -----------
'''Returns:	bool: call result'''
ep_gimbal.resume()

# ------ Subscribe to gimbal attitude angle information -------------
'''
Parameters:	
freq – enum: (1, 5, 10, 20, 50) Set the push frequency of data subscription data, in Hz
callback –
Callback function, return data (pitch_angle, yaw_angle, pitch_ground_angle, yaw_ground_angle):
pitch_angle:	Pitch axis angle relative to chassis
yaw_angle:	The angle of the yaw axis relative to the chassis
pitch_ground_angle:
 	Pitch axis angle at power-on time
yaw_ground_angle:
 	yaw axis angle at power-on time
args – variadic arguments
kw – keyword argument
Returns:	
bool: data subscription result'''
# ep_gimbal.sub_angle(freq=5, callback=None, *args, **kw)
ep_gimbal.sub_angle(freq=5, callback=None)

# -------- Control the gimbal to enter the sleep state -------------
'''Returns:	bool: call result'''
ep_gimbal.suspend()

# --------- Cancel Gimbal Attitude Angle Subscription --------------
'''Returns:	bool: the result of canceling the data subscription'''
ep_gimbal.unsub_angle()

# --------------------------------------------------------------------------------------------------------------------
# ------------------------------------ servo controls: ---------------------------------------------------------------
# --------------------------------------------------------------------------------------------------------------------

# ---------- Get the servo angle value -----------
'''Parameters:	index– int: [1, 3], servo number
Returns:	int servo angle'''
ep_servo.get_angle(index=1)

# ---------- Servo absolute position move ----------
'''
Parameters:	
index – int [1, 3], servo number
angle – int: [-180, 180], the rotation angle of the servo, unit (°)
Returns:	
action object'''
ep_servo.moveto(index=0, angle=0)

# ------------------ stop -------------------------
'''Parameters:	index– int: [1, 3], servo number
Return bool:	call result'''
ep_servo.pause(index=0)

# ------- Subscribe to servo angle information -------
'''
Parameters:	
freq – enum: (1, 5, 10, 20, 50) Set the push frequency of data subscription data, in Hz
callback –
Callback function, return data (valid[4], speed[4], angle[4]):
valid[4]:	4 servos online status
speed[4]:	Speed value of 4 servos
angle[4]:	Angle value of 4 servos
args – variadic arguments
kw – keyword argument
Returns:	
bool: data subscription result'''
# ep_servo.sub_servo_info(freq=5, callback=None, *args, **kw)
ep_servo.sub_servo_info(freq=5, callback=None)

# ----------- Unsubscribe the angle information of the servo -------------
'''return: bool: call result'''
ep_servo.unsub_servo_info()

# -------PWM (pulse width modulation) -------
# controls the duration of a high level of output during a certain period,
# and is broadly used to control LEDs, navigation gears, and more.

# UART:
# EP supports multiple access methods such as USB, Wi-Fi, and UART.


ep_robot.close()
