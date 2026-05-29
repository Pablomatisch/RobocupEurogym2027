# LEGO slot:0 autostart 

#import functions
from hub import port, motion_sensor, light_matrix
import runloop, motor, motor_pair, color_sensor, color, distance_sensor, math
from time import sleep

#define ports for better overview
#forward color
fc = port.A
#left color
lc = port.D
#right color
rc = port.C
#left motor
lm = port.F
#right motor
rm = port.E
#forward distance (sensor)
fd = port.B

#values for calibrating color sensors
CALIBRATION_MIN_VALID = 3
black_reflection = 60
white_reflection = 100
reflection_treshold = (black_reflection + white_reflection)/2

#set speeds
SPEED_STRAIGHT_FORWARD = 553
SPEED_TURN_HIGH = SPEED_STRAIGHT_FORWARD + 910
SPEED_TURN_LOW = - SPEED_STRAIGHT_FORWARD + 385
SPEED_SLOW = 70

#important values
TURN_180_TIME = 1.7/ 330 * SPEED_STRAIGHT_FORWARD
TURN_TIME = 0.45 / 330 * SPEED_STRAIGHT_FORWARD
OBSTACLE_DISTANCE = 40
CLEAR_DISTANCE = 200
WHEEL_DIAMETER = 5.2
last_color_right = "white"
last_color_left = "white"
last_direction_turned = ""
since_last_black_right = 100

#define the motor pair
motor_pair.pair(motor_pair.PAIR_1, lm, rm)

def color_is_black(port: int):
    return color_sensor.reflection(port) > reflection_treshold

def color_is_white(port: int):
    return color_sensor.reflection(port) < reflection_treshold

def color_is_green(port: int):
    r, g, b, intensity = color_sensor.rgbi(port)

    # avoid black / very dark readings
    if intensity < 300:
        return False

    #check if green is greater than red
    if g > r * 1.3 and g >= b * 1.02:
        return True

    return False

def color_is_silver(port: int):
    r, g, b, intensity = color_sensor.rgbi(port)

    #checks if everything is greater than 1000
    if r > 1015 and g > 1015 and b > 1015:
        print("silver detected")
        return True
    else:
        return False

def stop_motors():
    """
    Stops all current motor activity using the motor module
    """
    motor.stop(lm)
    motor.stop(rm)

def set_motors_straight_forward(velocity:int = SPEED_STRAIGHT_FORWARD):
    """
    Sets the motors to drive straight ahead using the motor module

    Args:
        velocity (int): Motor speed in degrees per second (default: SPEED_STRAIGHT_FORWARD)
    """
    motor.run(lm, -(velocity))
    motor.run(rm, velocity)

def set_motors_turn_right(high_velocity:int = SPEED_TURN_HIGH, low_velocity:int = SPEED_TURN_LOW):
    """
    Sets the motors to a right turn using the motor module

    Args:
        high_velocity (int): Motor speed in degrees per second (default: SPEED_TURN_HIGH)
        low_velocity (int): Motor speed in degrees per second (default: SPEED_TURN_LOW)
    """
    motor.run(lm, -(high_velocity))
    motor.run(rm, -(low_velocity))

def set_motors_turn_left(high_velocity:int = SPEED_TURN_HIGH, low_velocity:int = SPEED_TURN_LOW):
    """
    Sets the motors to a left turn using the motor module

    Args:
        high_velocity (int): Motor speed in degrees per second (default: SPEED_TURN_HIGH)
        low_velocity (int): Motor speed in degrees per second (default: SPEED_TURN_LOW)
    """
    motor.run(lm, low_velocity)
    motor.run(rm, high_velocity)

def deg_for_distance(distance_cm: float, wheel_diameter_cm: float = WHEEL_DIAMETER) -> int:
    """Convert distance in cm to motor rotation degrees."""
    circumference = math.pi * wheel_diameter_cm
    return int((distance_cm / circumference) * 360)


async def drive_straight(distance_cm: float,
                        velocity: int = SPEED_STRAIGHT_FORWARD,
                        stop_at_black: bool = False,
                        wheel_diameter_cm: float = WHEEL_DIAMETER,
                        kp: float = 2.0):
    """
    Drive straight for a given distance (cm) and speed (°/s),
    using gyro-based correction.

    Returns True when stopped by black line

    Args:
        distance_cm (float): Distance to travel in cm.
        velocity (int): Motor speed in degrees per second.
        stop_at_black (bool): Defines if the function should stop when detecting a black line
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
        if stop_at_black:
            if color_is_black(fc):
                print("stopped at black")
                motor_pair.stop(motor_pair.PAIR_1)
                set_motors_straight_forward()
                return True
        # SPIKE 3: tilt_angles()[0] gives yaw in deci-degrees with inverted sign
        yaw_deg = motion_sensor.tilt_angles()[0] * -0.1
        error = 0 - (yaw_deg * direction_forward_multiplicator)
        steer = int(max(-100, min(100, kp * error)))# Clamp steering between -100 and 100

        # move forward an set new steering correction
        motor_pair.move(motor_pair.PAIR_1, steer, velocity=abs(velocity)*direction_forward_multiplicator)
        moved_degrees = abs(motor.relative_position(lm))

    motor_pair.stop(motor_pair.PAIR_1)
    return False

async def rotate_degrees(rotate_degrees: float, stop_at_black: bool = False, velocity: int = SPEED_STRAIGHT_FORWARD):
    """
    Turns the robot for a several degrees.
    Args:
        degrees: amount of degrees to turn the robot (use negative values to turn right and positiv values to turn left)
    """
    global last_color_right
    global last_color_left
    global since_last_black_right
    # Reset the gyro
    motion_sensor.reset_yaw(0)
    steer = -100
    if (rotate_degrees < 0):
        steer = -steer
    rotated_degrees = 0

    while abs(rotate_degrees) > rotated_degrees:
        if stop_at_black:
            if color_is_black(fc):
                motor_pair.stop(motor_pair.PAIR_1)
                last_color_left = "white"
                last_color_right = "right"
                since_last_black_right += 1
                set_motors_straight_forward()
                return True
        motor_pair.move(motor_pair.PAIR_1, steer, velocity=velocity)
        rotated_degrees = abs(motion_sensor.tilt_angles()[0]*0.1)
        # calculate decellarateion relative to angle left to rotate
        rotate_degrees_left = abs(rotate_degrees) - rotated_degrees
    print("rotated", rotate_degrees, "degrees")
    motor_pair.stop(motor_pair.PAIR_1)
    return False

def update_last_colors():

    """ Checks and updates the last seen color of the left and right color sensors. """
    global last_color_right
    global last_color_left
    global since_last_black_right
    #save the last color the right sensor sees
    since_last_black_right += 1
    if (color_is_black(rc) and (last_color_right != "black") and last_color_right != "green" and not color_is_green(rc)):
        print("saved black")
        last_color_right = "black"
        since_last_black_right = 0
    if (color_is_white(rc) and (last_color_right != "white") and not color_is_green(rc)):
        print("saved white right")
        last_color_right = "white"
    if (last_color_right != "green") and color_is_green(rc):
        print("saved green")
        last_color_right = "green"

    #save the last color the left sensor sees
    if (color_is_black(lc) and (last_color_left != "black") and last_color_left != "green" and not color_is_green(lc)):
        last_color_left = "black"
    if (color_is_white(lc) and (last_color_left != "white") and not color_is_green(lc)):
        last_color_left = "white"
    if ((last_color_left != "green") and color_is_green(lc)):
        print("saved green")
        last_color_left = "green"

async def correct_line_path():
    """
    Corrects the current path of the robot to continue following the black line

    Also handles the end of the course
    """
    global last_direction_turned
    global since_last_black_right
    if (color_sensor.color(fc) == color.RED):
            await drive_straight(2)
            exit()
    #drive straight forward wenn forward color is black
    #if (color_is_black(fc)) and color_is_white(rc) and color_is_white(lc):
    if color_is_white(fc) and color_is_white(rc) and color_is_white(lc) and since_last_black_right <= 200:
        print(since_last_black_right)
        stop_motors()
        print("case")
        await drive_straight(-1)
        await rotate_degrees(-90, True)
        set_motors_straight_forward()
        print("case end")
    elif color_is_white(fc) and color_is_white(rc) and color_is_white(lc) and last_color_left == "black":
        stop_motors()
        print("case")
        await rotate_degrees(90, True)
        set_motors_straight_forward()

    elif color_is_white(fc) and color_is_white(rc) and color_is_white(lc):
        if last_direction_turned == "left":
            set_motors_turn_right()
        if last_direction_turned == "right":
            set_motors_turn_left()
    
    if color_is_white(rc) and color_is_white(lc):
        set_motors_straight_forward()
        #last_direction_turned == "forward"
    else:
        if color_is_black(lc) and color_is_black(rc):
            set_motors_turn_right()
        #turn left if left color is black
        if (color_is_black(lc)):
            set_motors_turn_left()
            last_direction_turned = "left"
           #turn right if right color is black
        if (color_is_black(rc)):
            set_motors_turn_right()
            last_direction_turned = "right"
        #drive forward to cross the goal line and the quit the program if forward color is red
        


def update_calibration():
    """updates the current calibration values based on new data from sensors"""
    global white_reflection
    global black_reflection
    global reflection_treshold
    for sensor in [fc, rc, lc]:
        #take multiple reading to smooth spikes
        readings = [color_sensor.reflection(sensor) for _ in range(5)]
        value = sum(readings) / len(readings)

        #ignore invalid reading
        if value < CALIBRATION_MIN_VALID:
            continue
        #update white if higher
        if value > white_reflection:
            white_reflection = value
            #print("updated white reflection to: ", white_reflection)
        #update black if lower
        if value < black_reflection:
            black_reflection = value
            #print("updated black reflection to: ", black_reflection)
    reflection_treshold = (white_reflection + black_reflection) / 2

async def main():
    #Linefollower workcycle and main function
    light_matrix.show_image(light_matrix.IMAGE_ARROW_N)
    set_motors_straight_forward()
    while True:
        #update_calibration()
        await correct_line_path()
        update_last_colors()
        #check_for_turns()
        #await check_for_zone()
        #await check_for_obstacles()
        

runloop.run(main())

