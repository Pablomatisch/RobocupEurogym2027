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
black_reflection = 75
white_reflection = 100
WHEEL_DIAMETER = 5.2
REFLECTION_TRESHOLD = (black_reflection + white_reflection)/2
last_color_right = "white"
last_color_left = "white"

TURN_180_TIME = 2.3
OBSTACLE_DISTANCE = 35
CLEAR_DISTANCE = 200

#set speeds
SPEED_STRAIGHT_FORWARD = 230
SPEED_TURN_HIGH = 220
SPEED_TURN_LOW = 240



def color_is_black(port: int):
    return color_sensor.reflection(port) < REFLECTION_TRESHOLD

def color_is_white(port: int):
    return color_sensor.reflection(port) > REFLECTION_TRESHOLD

def stop_motors():
    """
    Stops all current motor activity using the motor module
    """
    motor.stop(lm)
    motor.stop(rm)

def set_motors_straight_forward():
    """
    Sets the motors to drive straight ahead using the motor module
    """
    motor.run(lm, -(SPEED_STRAIGHT_FORWARD))
    motor.run(rm, SPEED_STRAIGHT_FORWARD)

def set_motors_turn_right():
    """
    Sets the motors to a right turn using the motor module
    """
    motor.run(lm, -(SPEED_TURN_HIGH))
    motor.run(rm, -(SPEED_TURN_LOW))

def set_motors_turn_left():
    """
    Sets the motors to a left turn using the motor module
    """
    motor.run(lm, SPEED_TURN_LOW)
    motor.run(rm, SPEED_TURN_HIGH)

def deg_for_distance(distance_cm: float, wheel_diameter_cm: float) -> int:
    """Convert distance in cm to motor rotation degrees."""
    circumference = math.pi * wheel_diameter_cm
    return int((distance_cm / circumference) * 360)


async def drive_straight(distance_cm: float,
                        velocity: int = SPEED_STRAIGHT_FORWARD,
                        wheel_diameter_cm: float = WHEEL_DIAMETER,
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
    direction_forward_multiplicator = distance_cm * velocity
    if (direction_forward_multiplicator > 0):
        direction_forward_multiplicator = 1
    else:
        direction_forward_multiplicator = -1

    moved_degrees = 0

    while moved_degrees < target_degrees:
        # SPIKE 3: tilt_angles()[0] gives yaw in deci-degrees with inverted sign
        yaw_deg = motion_sensor.tilt_angles()[0] * -0.1
        error = 0 - (yaw_deg * direction_forward_multiplicator)
        steer = int(max(-100, min(100, kp * error)))# Clamp steering between -100 and 100

        # move forward an set new steering correction
        motor_pair.move(motor_pair.PAIR_1, steer, velocity=abs(velocity)*direction_forward_multiplicator)
        moved_degrees = abs(motor.relative_position(lm))

    motor_pair.stop(motor_pair.PAIR_1)

async def rotate_degrees(rotate_degrees: float, velocity: int = SPEED_STRAIGHT_FORWARD):
    """
    Turns the robot for a several degrees.
    Args:
        degrees: amount of degrees to turn the robot (use negative values to turn right and positiv values to turn left)
    """
    # Reset the gyro
    motion_sensor.reset_yaw(0)
    steer = -100
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

def update_last_colors():
    """ Checks and updates the last seen color of the left and right color sensors. """
    global last_color_right
    global last_color_left
    #save the last color the right sensor sees
    if (color_is_black(rc) and (last_color_right != "black") and last_color_right != "green" and color_sensor.color(rc) != color.GREEN):
        last_color_right = "black"
    if (color_is_white(rc) and (last_color_right != "white") and color_sensor.color(rc) != color.GREEN):
        last_color_right = "white"
    if (last_color_right != "green") and color_sensor.color(rc) == color.GREEN:
        last_color_right = "green"

    #save the last color the left sensor sees
    if (color_is_black(lc) and (last_color_left != "black") and last_color_left != "green" and color_sensor.color(lc) != color.GREEN):
        last_color_left = "black"
    if (color_is_white(lc) and (last_color_left != "white") and color_sensor.color(lc) != color.GREEN):
        last_color_left = "white"
    if ((last_color_left != "green") and color_sensor.color(lc) == color.GREEN):
        last_color_left = "green"

def check_for_turns():
    """
    Checks for any green markings on the ground and lets the robot turn in the right direction if they follow up to white

    Attention: Requires the updateLastColor() function to be run in immediate advance in order to work properly
    """
    global last_color_left
    global last_color_right
    #check colors on the ground in case there is a turn
    if color_sensor.color(lc) == color.GREEN and color_sensor.color(rc) == color.GREEN:
        print("did a full turn")
        last_color_right = "green"
        last_color_left = "green"
        set_motors_turn_right()
        sleep(TURN_180_TIME)
        set_motors_straight_forward()
        sleep(0.2)
        set_motors_turn_right()

    #turn right if green follows directly to white
    if color_is_black(rc) and last_color_right != "black":
        if last_color_right == "green":
            print("turned right")
            set_motors_turn_right()
            sleep(0.3)
            set_motors_straight_forward()
            sleep(0.3)
            last_color_right = "green"
        
    #turn right if green follows directly to white
    if color_is_black(lc) and last_color_left != "black":
        if last_color_left == "green":
            print("turned left")
            set_motors_turn_left()
            sleep(0.3)
            set_motors_straight_forward()
            sleep(0.3)
            last_color_left = "black"

async def check_for_obstacles():
    """ Checks if there are obstacles in front of the robot and maneuvers around. """

#check the forward distance for any obstacle
    if distance_sensor.distance(fd) < OBSTACLE_DISTANCE and distance_sensor.distance(fd) != -1:
        print("detected object")
        #stop all movement
        stop_motors()
        #then turn to the right
        await rotate_degrees(-90)
        #drive forward
        await drive_straight(25)
        #turn back
        await rotate_degrees(86)
        #then drive forward and turn back once a while to check if reached the end of the obstacle and then end the cycle
        while True:
            await drive_straight(20)
            await rotate_degrees(90)
            await drive_straight(-1)
            await drive_straight(1)
            if distance_sensor.distance(fd) > CLEAR_DISTANCE:
                break
            await rotate_degrees(-90)
        await rotate_degrees(-90)
        await drive_straight(15)
        await rotate_degrees(90)
        await drive_straight(20)
        await rotate_degrees(-85)
        set_motors_straight_forward()

async def correct_line_path():
    """
    Corrects the current path of the robot to continue following the black line

    Also handles the end of the course
    """

    #drive straight forward wenn forward color is black
    if (color_is_black(fc)):
        set_motors_straight_forward()
    else:
        #if color_sensor.reflection(lc) > REFLECTION_TRESHOLD
        #turn left if left color is black
        if (color_is_black(lc)):
            set_motors_turn_left()
           #turn right if right color is black
        if (color_is_black(rc)):
            set_motors_turn_right()
        #drive forward to cross the goal line and the quit the program if forward color is red
        if (color_sensor.color(fc) is color.RED):
            await motor_pair.move_for_degrees(motor_pair.PAIR_1, 200, 0)
            quit


async def main():
    #Linefollower workcycle and main function
    while True:
        update_last_colors()
        check_for_turns()
        await check_for_obstacles()
        await correct_line_path()

runloop.run(main())

