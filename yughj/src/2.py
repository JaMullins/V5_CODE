# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       williamconner                                                #
# 	Created:      9/28/2023, 3:24:20 PM                                        #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #
# Library imports
import time
from vex import *

# configure devices 
brain=Brain()
controller = Controller()
# sensors

FWD = False
RVE = True

inertial = Inertial(Ports.PORT6)

motor_right_back_bottom =   Motor(Ports.PORT10, GearSetting.RATIO_36_1, FWD)
motor_right_back_top =      Motor(Ports.PORT9,  GearSetting.RATIO_36_1, RVE)
motor_right_front =         Motor(Ports.PORT8,  GearSetting.RATIO_36_1, FWD)
right_motors = MotorGroup(motor_right_back_bottom, motor_right_back_top, motor_right_front)

motor_left_back_bottom =    Motor(Ports.PORT11, GearSetting.RATIO_36_1, RVE)
motor_left_back_top =       Motor(Ports.PORT12, GearSetting.RATIO_36_1, FWD)
motor_left_front =          Motor(Ports.PORT7,  GearSetting.RATIO_36_1, RVE) 
left_motors = MotorGroup(motor_left_back_bottom, motor_left_back_top, motor_left_front)

drive_train = SmartDrive(
    lm = left_motors,
    rm = right_motors,
    g = inertial,
    wheelTravel = 319,
    trackWidth = 370,
    wheelBase = 246,
    units = DistanceUnits.MM)



is_rev = False

def driver_controlled():
    global left_motors, right_motors, intake, catapult, intake_arm, wings, controller, rotational, down_catapult_angle, backstop_catapult_angle, brain, rotational, wings_extended, intake_arm_extended, is_rev
    controller.rumble("-")
    
    def toggle_rev():
        global is_rev
        is_rev = not is_rev
    controller.buttonB.pressed(toggle_rev)

    while True:
        # motion
        forward_motion = controller.axis3.position()

        forward_motion_decimal = float(forward_motion/100)
        rotational_multiplier = .75 + (.25 * (1 - forward_motion_decimal))
        rotational_motion = controller.axis1.position() * rotational_multiplier

        net_left = (forward_motion + rotational_motion)
        net_right = (forward_motion - rotational_motion)
        
        if is_rev:
            forward_motion *= -1
            
        left_motors.set_velocity((forward_motion + rotational_motion), PERCENT)
        right_motors.set_velocity((forward_motion - rotational_motion), PERCENT)
        left_motors.spin(FORWARD)
        right_motors.spin(FORWARD)

        # catapult
        time.sleep(0.02)

def set_auton_setting():
    pass

def autonomous():
    pass

def skills():
    pass

### Get Auton Setting
set_auton_setting()

autonomous()
driver_controlled()

### Create competition object
competition = Competition(driver_controlled, autonomous)