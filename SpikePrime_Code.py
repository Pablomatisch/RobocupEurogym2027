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
black_reflection = 100
white_reflection = 0
reflection_treshold = (black_reflection + white_reflection)/2

# image names
ARROW_RIGHT = light_matrix.IMAGE_ARROW_W
ARROW_LEFT = light_matrix.IMAGE_ARROW_E
ARROW_FRONT = light_matrix.IMAGE_ARROW_S
ARROW_BACK = light_matrix.IMAGE_ARROW_N

#set speeds
SPEED_STRAIGHT_FORWARD = 340
SPEED_TURN_HIGH = SPEED_STRAIGHT_FORWARD
SPEED_TURN_LOW = 120
SPEED_SLOW = 160

#important values
TURN_180_TIME = 1.7/ 330 * SPEED_STRAIGHT_FORWARD
TURN_TIME = 0.45 / 330 * SPEED_STRAIGHT_FORWARD
OBSTACLE_DISTANCE = 80
CLEAR_DISTANCE = 200
WHEEL_DIAMETER = 5.2
last_color_right = "white"
last_color_left = "white"
SILVER_TRESHOLD = 1017

#for zone handling
ZONE_TRESHOLD = 220
previous_distance = 10000

#define the motor pair
motor_pair.pair(motor_pair.PAIR_1, lm, rm)

def color_is_black(port: int):
    return color_sensor.reflection(port) < reflection_treshold

def color_is_white(port: int):
    return color_sensor.reflection(port) > reflection_treshold

def color_is_green(port: int):
    r, g, b, intensity = color_sensor.rgbi(port)

    # avoid black / very dark readings
    if intensity < 300:
        return False

    #check if green is greater than red
    if g > r * 1.2 and g >= b * 1.01:
        return True

    return False

def color_is_silver(port: int):
    r, g, b, intensity = color_sensor.rgbi(port)

    #checks if everything is greater than 1000
    if r > SILVER_TRESHOLD and g > SILVER_TRESHOLD and b > SILVER_TRESHOLD and intensity > 1010:
        #print("silver detected")
        return True
    else:
        return False

def stop_motors():
    """
    Stops all current motor activity using the motor module
    """
    motor.stop(lm)
    motor.stop(rm)

def set_motors_straight_forward(velocity:float = SPEED_STRAIGHT_FORWARD):
    """
    Sets the motors to drive straight ahead using the motor module

    Args:
        velocity (int): Motor speed in degrees per second (default: SPEED_STRAIGHT_FORWARD)
    """
    motor.run(lm, -int(velocity))
    motor.run(rm, int(velocity))

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
                motor_pair.stop(motor_pair.PAIR_1)
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

async def rotate_degrees(rotate_degrees: float, velocity: int = SPEED_STRAIGHT_FORWARD, stop_at_black: bool = False):
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
        if stop_at_black:
            if color_is_black(fc):
                motor_pair.stop(motor_pair.PAIR_1)
                return True
        motor_pair.move(motor_pair.PAIR_1, steer, velocity=velocity)
        rotated_degrees = abs(motion_sensor.tilt_angles()[0]*0.1)
    print("rotated", rotate_degrees, "degrees")
    motor_pair.stop(motor_pair.PAIR_1)
    return False

def update_last_colors():
    """ Checks and updates the last seen color of the left and right color sensors. """
    global last_color_right
    global last_color_left
    #save the last color the right sensor sees
    if (color_is_black(rc) and (last_color_right != "black") and last_color_right != "green" and not color_is_green(rc)):
        last_color_right = "black"
    if (color_is_white(rc) and (last_color_right != "white") and not color_is_green(rc)):
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

async def check_for_turns():
    """
    Checks for any green markings on the ground and lets the robot turn in the right direction if they follow up to white

    Attention: Requires the updateLastColor() function to be run in immediate advance in order to work properly
    """
    global last_color_left
    global last_color_right
    #check colors on the ground in case there is a turn
    if color_is_green(lc) and color_is_green(rc):
        print("did a full turn")
        light_matrix.show_image(ARROW_BACK)
        last_color_right = "green"
        last_color_left = "green"
        set_motors_turn_right()
        sleep(TURN_180_TIME)
        set_motors_straight_forward()
        sleep(0.2)
        set_motors_turn_right()
        light_matrix.show_image(ARROW_FRONT)

    #turn right if black follows to green
    if color_is_black(rc) and last_color_right == "green" and not color_is_green(rc):
            print("turned right")
            light_matrix.show_image(ARROW_RIGHT)
            await drive_straight(8)
            await rotate_degrees(-15, SPEED_SLOW)
            await rotate_degrees(-75, SPEED_SLOW, True)
            # set_motors_turn_right()
            # sleep(TURN_TIME)
            # set_motors_straight_forward()
            # sleep(0.3)
            last_color_right = "black"
            light_matrix.show_image(ARROW_FRONT)
        
    #turn right if black follows to green
    if color_is_black(lc) and last_color_left == "green" and not color_is_green(lc):
            print("turned left")
            light_matrix.show_image(ARROW_LEFT)
            await drive_straight(8)
            await rotate_degrees(15, SPEED_SLOW)
            await rotate_degrees(75, SPEED_SLOW, True)
            # set_motors_turn_left()
            # sleep(TURN_TIME)
            # set_motors_straight_forward()
            # sleep(0.3)
            last_color_left = "black"
            light_matrix.show_image(ARROW_FRONT)

async def check_for_obstacles():
    """ Checks if there are obstacles in front of the robot and maneuvers around. """
    count = 3

    if distance_sensor.distance(fd) < OBSTACLE_DISTANCE and distance_sensor.distance(fd) != -1:
        #wait for a short time to make sure its not a false positive
        sleep(0.2)
        if distance_sensor.distance(fd) < OBSTACLE_DISTANCE and distance_sensor.distance(fd) != -1:
            print("obstacle detected")
            light_matrix.show_image(light_matrix.IMAGE_SQUARE)
            await rotate_degrees(-90, SPEED_SLOW)
            await drive_straight(25, SPEED_SLOW)
            await rotate_degrees(90, SPEED_SLOW)
            while True:
                if await drive_straight(45, SPEED_SLOW, True):
                    break
                await rotate_degrees(90, SPEED_SLOW)
                count -= 1
                if count == 0:
                    set_motors_straight_forward()
                    break
            await rotate_degrees(-15)
            light_matrix.show_image(ARROW_FRONT)

async def check_for_zone():
    global ZONE_TRESHOLD
    global previous_distance
    """Checks if entered the zone and tries to escape it"""
    if color_is_silver(fc) and distance_sensor.distance(fd) > 500:
        print("distance and color from zone:", distance_sensor.distance(fd))
        light_matrix.show_image(light_matrix.IMAGE_DIAMOND)
        await drive_straight(1)
        sleep(0.5)
        await drive_straight(1)
        if color_is_silver(rc):
            if color_is_white(fc) and color_is_white(rc) and color_is_white(lc):
                print("entered zone")
                await drive_straight(10)
                await rotate_degrees(10)
                await drive_straight(20)
                await rotate_degrees(-90)
                while True:
                    reading = distance_sensor.distance(fd)
                    if reading > previous_distance + ZONE_TRESHOLD:
                        await rotate_degrees(8)
                        if await drive_straight(55, SPEED_SLOW, True):
                            break
                    previous_distance = reading
                    await rotate_degrees(2, SPEED_SLOW)
        else:
            light_matrix.show_image(ARROW_FRONT)

async def correct_line_path():
    """
    Corrects the current path of the robot to continue following the black line

    Also handles the end of the course
    """

    #drive straight forward wenn forward color is black
    if (color_is_black(fc)) and color_is_white(rc) and color_is_white(lc):
        set_motors_straight_forward()
    else:
        #turn left if left color is black
        if (color_is_black(lc)):
            set_motors_turn_left()
           #turn right if right color is black
        if (color_is_black(rc)):
            set_motors_turn_right()
        #drive forward to cross the goal line and the quit the program if forward color is red
        if (color_sensor.color(fc) == color.RED):
            await drive_straight(2)
            exit()

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
    light_matrix.show_image(ARROW_FRONT)
    set_motors_straight_forward()
    while True:
        update_calibration()
        update_last_colors()
        await check_for_turns()
        await check_for_zone()
        await check_for_obstacles()
        await correct_line_path()

runloop.run(main())

