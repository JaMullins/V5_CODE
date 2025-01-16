# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       jack mullins                                                 #
# 	Created:      1/15/2025 2:11 PM                                            #
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
first_intake_motor = Motor(Ports.PORT16, GearSetting.RATIO_6_1, REV)

## Pneumatics
intake_pneumatics = DigitalOut(brain.three_wire_port.h)
clamp_pneumatics = DigitalOut(brain.three_wire_port.a)
arm_pneumatics = DigitalOut(brain.three_wire_port.b)



intertial_sensor = Inertial(Ports.PORT14)


"""

## Sensors, require calibration

gps_sensor = Gps(Ports.PORT1)"""


# Logic
in_reverse = False
clamp = False
arm = False
is_intake_go = False
slow = False



"""
def toggle_intake():
    global intake_pneumatics
    if intake_pneumatics.value():
        intake_pneumatics.close()
    else:
        intake_pneumatics.open()"""

def driver_controlled():
    global brain, controller, in_reverse, drive_train, left_motors, right_motors, second_intake_motor, intake_pneumatics, intertial_sensor, gps_sensor, clamp, is_intake_go, arm, slow
    controller.rumble('-');

    def toggle_clamp():
        global clamp
        clamp = not clamp

    def slow_down():
        global slow
        slow = not slow

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
    controller.buttonA.pressed(slow_down)

    controller.buttonR1.pressed(intake_normal)
    controller.buttonR2.pressed(intake_slow)
    controller.buttonL2.pressed(intake_reverse)
    controller.buttonL1.pressed(intake_first_stage)



    #controller.buttonB.pressed(toggle_intake)
    # controller.buttonX.pressed(toggle_reverse)
    while not time.sleep(.02):
        global in_reverse, slow
        # Motion
        forward_motionL = controller.axis3.position()
        forward_motionR = controller.axis2.position()

        net_left_motion = forward_motionL
        net_right_motion = forward_motionR

        if slow:
            right_motors.set_velocity(net_left_motion, RPM)
            left_motors.set_velocity(net_right_motion, RPM)
        else:
            right_motors.set_velocity(net_left_motion, PERCENT)
            left_motors.set_velocity(net_right_motion, PERCENT)
        
        if in_reverse:
            right_motors.spin(REVERSE)
            left_motors.spin(REVERSE)
        else:
            left_motors.spin(FORWARD)
            right_motors.spin(FORWARD)

        clamp_pneumatics.set(clamp)
        arm_pneumatics.set(arm)
        

        

        # Intake
        # intake_standardized_velocity = int(controller.buttonR2.pressing()) - int(controller.buttonR1.pressing())
        # intake_motor.spin(FORWARD, 100 * intake_standardized_velocity, PERCENT)


def autonomous():
    global brain, controller, drive_train, second_intake_motor, intake_pneumatics, intertial_sensor, gps_sensorm, left_motors, right_motors, clamp_pneumatics
    intertial_sensor.calibrate()
    intertial_sensor.set_rotation(0)
    intertial_sensor.set_heading(0)


    drive_train.set_drive_velocity(31, PERCENT)
    drive_train.drive(FORWARD)
    time.sleep(1)
    drive_train.stop()
    clamp_pneumatics.set(True)
    drive_train.set_turn_velocity(31, PERCENT)
    drive_train.turn_for(45, DEGREES)

    drive_train.set_drive_velocity(100,PERCENT)
    drive_train.drive(FORWARD)
    time.sleep(2)
    drive_train.stop()

    clamp_pneumatics.set(False)
    drive_train.drive(REVERSE)
    time.sleep(1.5)
    drive_train.stop()






    


#competition = Competition(driver_controlled, autonomous)

#autonomous() ## UNCOMMENT THIS FOR AN AUTON RUN

#driver_controlled() ## UNCOMMENT THIS FOR A DRIVER CONTROLLED RUN
