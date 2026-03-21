# LEGO slot:0 autostart

#import functions
from hub import port, motion_sensor
import runloop, motor, motor_pair, color_sensor, color, distance_sensor, math
from time import sleep

#define ports for better overview
#forward color
fc = port.B
#left color
lc = port.E
#right color
rc = port.F
#left motor
lm = port.A
#right motor
rm = port.D
#forward distance (sensor)
fd = port.C

#define the motor pair
motor_pair.pair(motor_pair.PAIR_1, lm, rm)

#define important values
blackReflection = 80
whiteReflection = 100
wheel_diameter = 5.2
ReflectionTreshold = (blackReflection + whiteReflection)/2
#set speeds
speedStraightForward = 230
speedTurnHigh = 220
speedTurnLow = 240
#important values
lastColorRight = "white"
lastColorLeft = "white"


def colorIsBlack(port: int):
    return color_sensor.reflection(port) < ReflectionTreshold

def colorIsWhite(port: int):
    return color_sensor.reflection(port) > ReflectionTreshold

def stopMotors():
    """
    Stops all current motor activity using the motor module
    """
    motor.stop(lm)
    motor.stop(rm)

def setMotorsStraightForward():
    """
    Sets the motors to drive straight ahead using the motor module
    """
    motor.run(lm, -(speedStraightForward))
    motor.run(rm, speedStraightForward)

def setMotorsTurnRight():
    """
    Sets the motors to a right turn using the motor module
    """
    motor.run(lm, -(speedTurnHigh))
    motor.run(rm, -(speedTurnLow))

def setMotorsTurnLeft():
    """
    Sets the motors to a left turn using the motor module
    """
    motor.run(lm, speedTurnLow)
    motor.run(rm, speedTurnHigh)

def deg_for_distance(distance_cm: float, wheel_diameter_cm: float) -> int:
    """Convert distance in cm to motor rotation degrees."""
    circumference = math.pi * wheel_diameter_cm
    return int((distance_cm / circumference) * 360)


async def drive_straight(distance_cm: float,
                        velocity: int = speedStraightForward,
                        wheel_diameter_cm: float = wheel_diameter,
                        kp: float = 2.0,
                        step_deg: int = 60):
    """
    Drive straight for a given distance (cm) and speed (°/s),
    using gyro-based correction.

    Args:
        distance_cm (float): Distance to travel in cm.
        velocity (int): Motor speed in degrees per second.
        wheel_diameter_cm (float): Wheel diameter in cm.
        kp (float): Proportional gain for gyro correction (default 2.0).
        step_deg (int): Step size in motor degrees per correction cycle.
    """

    # Reset the gyro
    motion_sensor.reset_yaw(0)

    # Reset motor degrees
    motor.reset_relative_position(lm, 0)

    target_degrees = deg_for_distance(abs(distance_cm), wheel_diameter_cm)
    isDirectionForward = distance_cm * velocity
    if (isDirectionForward > 0):
        isDirectionForward = 1
    else:
        isDirectionForward = -1

    moved_degrees = 0

    while moved_degrees < target_degrees:
        # SPIKE 3: tilt_angles()[0] gives yaw in deci-degrees with inverted sign
        yaw_deg = motion_sensor.tilt_angles()[0] * -0.1
        error = 0 - (yaw_deg * isDirectionForward)
        steer = int(max(-100, min(100, kp * error)))# Clamp steering between -100 and 100

        # move forward an set new steering correction
        motor_pair.move(motor_pair.PAIR_1, steer, velocity=abs(velocity)*isDirectionForward)
        moved_degrees = abs(motor.relative_position(lm))

    motor_pair.stop(motor_pair.PAIR_1)

async def rotate_degrees(rotate_degrees: float, velocity: int = speedStraightForward):
    """
    Turns the robot for a several degrees.
    Args:
        degrees: amount of degrees to turn the robot (use negative values to turn right and positiv values to turn left)
    """
    # Reset the gyro
    motion_sensor.reset_yaw(0)
    steer = -100;
    if (rotate_degrees < 0):
        steer = -steer
    rotated_degrees = 0

    while abs(rotate_degrees) > rotated_degrees:
        motor_pair.move(motor_pair.PAIR_1, steer, velocity=velocity)
        rotated_degrees = abs(motion_sensor.tilt_angles()[0]*0.1)
        # calculate decellarateion relative to angle left to rotate
        rotate_degrees_left = abs(rotate_degrees) - rotated_degrees
    print("rotated", rotate_degrees, "degrees")
    motor_pair.stop(motor_pair.PAIR_1)

def updateLastColors():
    """ Checks and updates the last seen color of the left and right color sensors. """
    global lastColorRight
    global lastColorLeft
    #save the last color the right sensor sees
    if (colorIsBlack(rc) and (lastColorRight != "black") and lastColorRight != "green" and color_sensor.color(rc) != color.GREEN):
        lastColorRight = "black"
    if (colorIsWhite(rc) and (lastColorRight != "white") and color_sensor.color(rc) != color.GREEN):
        lastColorRight = "white"
    if (lastColorRight != "green") and color_sensor.color(rc) == color.GREEN:
        lastColorRight = "green"

    #save the last color the left sensor sees
    if (colorIsBlack(lc) and (lastColorLeft != "black") and lastColorLeft != "green" and color_sensor.color(lc) != color.GREEN):
        lastColorLeft = "black"
    if (colorIsWhite(lc) and (lastColorLeft != "white") and color_sensor.color(lc) != color.GREEN):
        lastColorLeft = "white"
    if ((lastColorLeft != "green") and color_sensor.color(lc) == color.GREEN):
        lastColorLeft = "green"

def checkForTurns():
    """
    Checks for any green markings on the ground and lets the robot turn in the right direction if they follow up to white

    Attention: Requires the updateLastColor() function to be run in immediate advance in order to work properly
    """
    global lastColorLeft
    global lastColorRight
    #check colors on the ground in case there is a turn
    if color_sensor.color(lc) == color.GREEN and color_sensor.color(rc) == color.GREEN:
        print("did a full turn")
        lastColorRight = "green"
        lastColorLeft = "green"
        setMotorsTurnRight()
        sleep(2.3)
        setMotorsStraightForward()
        sleep(0.2)
        setMotorsTurnRight()

    #turn right if green follows directly to white
    if colorIsBlack(rc) and lastColorRight != "black":
        if lastColorRight == "green":
            print("turned right")
            setMotorsTurnRight()
            sleep(0.3)
            setMotorsStraightForward()
            sleep(0.3)
            lastColorRight = "green"
        
    #turn right if green follows directly to white
    if colorIsBlack(lc) and lastColorLeft != "black":
        if lastColorLeft == "green":
            print("turned left")
            setMotorsTurnLeft()
            sleep(0.3)
            setMotorsStraightForward()
            sleep(0.3)
            lastColorLeft = "black"

async def checkForObstacles():
    """ Checks if there are obstacles in front of the robot and maneuvers around. """

#check the forward distance for any obstacle
    if distance_sensor.distance(fd) < 45 and distance_sensor.distance(fd) is not -1:
        print("detected object")
        distance_to_object = 20
        #stop all movement
        stopMotors()
        #then turn to the right
        await rotate_degrees(-90)
        #drive forward
        await drive_straight(20)
        #turn back
        await rotate_degrees(90)
        #then drive forward and turn back once a while to check if reached the end of the obstacle and then end the cycle
        while True:
            await drive_straight(distance_to_object)
            await rotate_degrees(90)
            await drive_straight(-1)
            await drive_straight(1)
            if distance_sensor.distance(fd) > 200:
                break
            distance_to_object = distance_to_object / 2
            await rotate_degrees(-90)
        await drive_straight(18)
        await rotate_degrees(-80)
        setMotorsStraightForward()

async def correctLinePath():
    """
    Corrects the current path of the robot to continue following the black line

    Also handles the end of the course
    """

    #drive straight forward wenn forward color is black
    if (colorIsBlack(fc)):
        setMotorsStraightForward()
    else:
        #if color_sensor.reflection(lc) > ReflectionTreshold
        #turn left if left color is black
        if (colorIsBlack(lc)):
            setMotorsTurnLeft()
           #turn right if right color is black
        if (colorIsBlack(rc)):
            setMotorsTurnRight()
        #drive forward to cross the goal line and the quit the program if forward color is red
        if (color_sensor.color(fc) is color.RED):
            await motor_pair.move_for_degrees(motor_pair.PAIR_1, 200, 0)
            quit


async def main():
    #Linefollower workcycle and main function
    while True:
        updateLastColors()
        checkForTurns()
        await checkForObstacles()
        await correctLinePath()

runloop.run(main())

