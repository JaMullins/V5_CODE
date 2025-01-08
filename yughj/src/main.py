# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       frankhowell                                                  #
# 	Created:      10/23/2024, 8:17:33 AM                                       #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
import time
from vex import *

# Part definitions
brain = Brain()
controller = Controller()

## Drive Train
REV = False
FWD = True

left_motors = MotorGroup(
    Motor(Ports.PORT18, GearSetting.RATIO_6_1, REV), # l. front
    Motor(Ports.PORT19, GearSetting.RATIO_6_1, REV), # l. middle
    Motor(Ports.PORT20, GearSetting.RATIO_6_1, REV), # l. back
)
right_motors = MotorGroup(
    Motor(Ports.PORT11, GearSetting.RATIO_6_1, FWD), # r. front
    Motor(Ports.PORT12, GearSetting.RATIO_6_1, FWD), # r. middle
    Motor(Ports.PORT13, GearSetting.RATIO_6_1, FWD), # r. back
)
 
drive_train = DriveTrain(
    lm = left_motors,
    rm = right_motors,

    wheelTravel = 0,
    trackWidth  = 0,
    wheelBase   = 0,
    units= DistanceUnits.MM,

    externalGearRatio= 1.0,
)

##Subsystem Motor(s)
second_intake_motor = Motor(Ports.PORT15)
first_intake_motor = Motor(Ports.PORT16)

## Pneumatics
intake_pneumatics = DigitalOut(brain.three_wire_port.h)
clamp_pneumatics = DigitalOut(brain.three_wire_port.a)
arm_pneumatics = DigitalOut(brain.three_wire_port.b)
"""

## Sensors, require calibration
intertial_sensor = Inertial(Ports.PORT1)
gps_sensor = Gps(Ports.PORT1)"""


# Logic
in_reverse = False
clamp = True
arm = False
is_intake_go = False


"""
def toggle_intake():
    global intake_pneumatics
    if intake_pneumatics.value():
        intake_pneumatics.close()
    else:
        intake_pneumatics.open()"""

def driver_controlled():
    global brain, controller, in_reverse, drive_train, left_motors, right_motors, second_intake_motor, intake_pneumatics, intertial_sensor, gps_sensor, clamp, is_intake_go, arm
    controller.rumble('-');

    def toggle_clamp():
        global clamp
        clamp = not clamp

    def toggle_reverse():
        global in_reverse
        in_reverse = not in_reverse

    def toggle_arm():
        global arm
        arm = not arm
    
    def intake_normal():
        global is_intake_go
        is_intake_go = not is_intake_go
        if is_intake_go:
            second_intake_motor.set_velocity(-250, RPM)
            second_intake_motor.spin(FORWARD)
            first_intake_motor.set_velocity(200, RPM)
            first_intake_motor.spin(FORWARD)
        else:
            first_intake_motor.set_velocity(0, RPM)
            second_intake_motor.set_velocity(0,RPM)

    def intake_slow():
        global is_intake_go
        is_intake_go = not is_intake_go
        if is_intake_go:
            second_intake_motor.set_velocity(-50, RPM)
            second_intake_motor.spin(FORWARD)
            first_intake_motor.set_velocity(200, RPM)
            first_intake_motor.spin(FORWARD)
        else:
            first_intake_motor.set_velocity(0, RPM)
            second_intake_motor.set_velocity(0,RPM)

    def intake_reverse():
        global is_intake_go
        is_intake_go = not is_intake_go
        if is_intake_go:
            second_intake_motor.set_velocity(100, RPM)
            second_intake_motor.spin(FORWARD)
            first_intake_motor.set_velocity(-80, RPM)
            first_intake_motor.spin(FORWARD)
        else:
            first_intake_motor.set_velocity(0, RPM)
            second_intake_motor.set_velocity(0,RPM)

    def intake_first_stage():
        global is_intake_go
        is_intake_go = not is_intake_go
        if is_intake_go:
            first_intake_motor.set_velocity(200, RPM)
            first_intake_motor.spin(FORWARD)
        else:
            first_intake_motor.set_velocity(0, RPM)
            second_intake_motor.set_velocity(0,RPM)

    controller.buttonX.pressed(toggle_clamp)
    controller.buttonY.pressed(toggle_arm)
    controller.buttonB.pressed(toggle_reverse)

    controller.buttonR1.pressed(intake_normal)
    controller.buttonR2.pressed(intake_slow)
    controller.buttonL2.pressed(intake_reverse)
    controller.buttonL1.pressed(intake_first_stage)



    #controller.buttonB.pressed(toggle_intake)
    # controller.buttonX.pressed(toggle_reverse)
    while not time.sleep(.02):
        # Motion
        forward_motionL = controller.axis3.position()

        forward_motion_as_decimal = float((forward_motionL)/100)
        rotational_multiplier = .75 + (.25 * (1 - forward_motion_as_decimal)) # makes the bot less sensative to turns at high speeds, making it easier to control
        rotational_motion = controller.axis1.position() * rotational_multiplier

        forward_motionR = controller.axis2.position()

        forward_motion_as_decimal = float((forward_motionR)/100)
        rotational_multiplier = .75 + (.25 * (1 - forward_motion_as_decimal)) # makes the bot less sensative to turns at high speeds, making it easier to control
        rotational_motion = controller.axis1.position() * rotational_multiplier

        net_left_motion = forward_motionL
        net_right_motion = forward_motionR
        
        right_motors.set_velocity(net_left_motion, PERCENT) ## RENAME THESE VARIABLE TYPES LATER
        left_motors.set_velocity(net_right_motion, PERCENT) 
        left_motors.spin(FORWARD)
        right_motors.spin(FORWARD)

        clamp_pneumatics.set(clamp)
        arm_pneumatics.set(arm)
        

        

        # Intake
        # intake_standardized_velocity = int(controller.buttonR2.pressing()) - int(controller.buttonR1.pressing())
        # intake_motor.spin(FORWARD, 100 * intake_standardized_velocity, PERCENT)


def autonomous():
    global brain, controller, drive_train, second_intake_motor, intake_pneumatics, intertial_sensor, gps_sensor

    raise NotImplementedError()


# competition = Competition(driver_controlled, autonomous)

driver_controlled()
