# LEGO slot:0 autostart 

# imports for robot class
from hub import motion_sensor, light_matrix
import motor, motor_pair, color_sensor, color, distance_sensor, math
from hub import port
import runloop
from time import sleep

class Hub:
    """
    Class for the hub, which contains all the ports and constants for the robot, as well as some helper functions for controlling the motors and sensors.
    """
    def __init__(self, 
                 forward_color_port, 
                 left_color_port, 
                 right_color_port, 
                 left_motor_port, 
                 right_motor_port, 
                 forward_distance_port,
                 obstacle_distance,
                 wheel_diameter,
                 default_reflection_treshold,
                 silver_threshold,
                 speed_straight_forward,
                 speed_turn_high,
                 speed_turn_low,
                 speed_slow):
        self.forward_color_port = forward_color_port # port of forward color sensor
        self.left_color_port = left_color_port # port of left color sensor
        self.right_color_port = right_color_port # port of right color sensor
        self.left_motor_port = left_motor_port # port of left motor
        self.right_motor_port = right_motor_port # port of right motor
        self.forward_distance_port = forward_distance_port # port of forward distance sensor
        self.obstacle_distance = obstacle_distance # distance at which an obstacle is detected as such
        self.wheel_diameter = wheel_diameter # diameter of the wheels, used for calculate distances and turns
        self.default_reflection_treshold = default_reflection_treshold # threshold for distinguishing between black and white surfaces
        self.silver_threshold = silver_threshold # threshold for detecting silver surfaces, used for zone handling
        self.speed_straight_forward = speed_straight_forward # speed for driving straight forward
        self.speed_turn_high = speed_turn_high # speed for the faster motor when turning
        self.speed_turn_low = speed_turn_low # speed for the slower motor when turning
        self.speed_slow = speed_slow # speed for slow movements, e.g. for some turns or for driving in the zone
        self.motor_pair = motor_pair.pair(motor_pair.PAIR_1, self.left_motor_port, self.right_motor_port) # motor pair for easier control of both motors at the same time



    #---------------------------------------------------------
    #
    # IMPORTANT HELPER FUNCTIONS
    #
    #---------------------------------------------------------

    
    def distance_in_mm(self, port = None):

        """
        Gets the distance in mm from the distance sensor on the given port.
        Args:
            port: the port of the distance sensor to check (default: forward distance sensor)
        """

        port = port or self.forward_distance_port
        return distance_sensor.distance(port)
    

    def deg_for_distance(self, distance_cm: float , wheel_diameter_cm = None) -> int:

        """
        Convert distance in cm to motor rotation degrees.
        Args:
            distance_cm: distance in cm to convert
            wheel_diameter_cm: diameter of the wheels in cm (default: self.wheel_diameter)
        """

        wheel_diameter_cm = wheel_diameter_cm or self.wheel_diameter
        circumference = math.pi * wheel_diameter_cm
        return int((distance_cm / circumference) * 360) 


    def show_image(self, image = None):

        """
        Shows an image on the light matrix. 
        Possible values for image are

          "arrow_front" for a an arrow pointing forward (default)

          "arrow_back" for a an arrow pointing backwards

          "arrow_left"  for a an arrow pointing left

          "arrow_right" for a an arrow pointing right

          "square" for a square

          "diamond" for a diamond
        """

        if image == "arrow_front":
            final_image = light_matrix.IMAGE_ARROW_S
        elif image == "arrow_back":
            final_image = light_matrix.IMAGE_ARROW_N
        elif image == "arrow_left":
            final_image = light_matrix.IMAGE_ARROW_E
        elif image == "arrow_right":
            final_image = light_matrix.IMAGE_ARROW_W
        elif image == "square":
            final_image = light_matrix.IMAGE_SQUARE
        elif image == "diamond":
            final_image = light_matrix.IMAGE_DIAMOND
        else:
            final_image = light_matrix.IMAGE_ARROW_S
        light_matrix.show_image(final_image)
        


    #---------------------------------------------------------
    #
    # HELPER FUNCTIONS FOR SENSORS
    #
    #---------------------------------------------------------

    def color_is_black(self, port = None, default_reflection_treshold = None):

        """
        Checks if the color sensor on the given port detects black based on the reflection value and the default reflection threshold.
        Args:
            port: the port of the color sensor to check (default: forward color sensor)
            default_reflection_treshold: the reflection threshold to use for distinguishing between black and white (default: default_reflection_treshold)
        """

        port = port or self.forward_color_port
    
        if default_reflection_treshold is None:
            default_reflection_treshold = self.default_reflection_treshold
        else:
            r, g , b, intensity = color_sensor.rgbi(port)
            if intensity < default_reflection_treshold:
                return True
            else:
                return False
        return (color_sensor.reflection(port) < default_reflection_treshold and not self.color_is_green(port) and not self.color_is_silver(port) and not self.color_is_red(port))

    def color_is_white(self, port = None, default_reflection_treshold = None):

        """ 
        Checks if the color sensor on the given port detects white based on the reflection value and the default reflection threshold.
        Args:
            port: the port of the color sensor to check (default: forward color sensor)
            default_reflection_treshold: the reflection threshold to use for distinguishing between black and white (default: default_reflection_treshold)
        """

        port = port or self.forward_color_port
        default_reflection_treshold = default_reflection_treshold or self.default_reflection_treshold
        return (color_sensor.reflection(port) > default_reflection_treshold and not self.color_is_green(port) and not self.color_is_silver(port) and not self.color_is_red(port))


    def color_is_green(self, port = None):
        
        """ 
        Checks if the color sensor on the given port detects green based on the RGB values.
        Args:
            port: the port of the color sensor to check (default: forward color sensor)
        """

        port = port or self.forward_color_port
        # take 3 readings and average them to avoid false positives
        r_total = 0
        g_total = 0
        b_total = 0
        intensity_total = 0
        for _ in range(3):
            r, g, b, intensity = color_sensor.rgbi(port)
            r_total += r
            g_total += g
            b_total += b
            intensity_total += intensity

        r = r_total / 3
        g = g_total / 3
        b = b_total / 3
        intensity = intensity_total / 3

        # avoid black / very dark readings
        if intensity < 140 or intensity > 700:
            return False or color_sensor.color(port) == color.GREEN

        #check if green is greater than red
        if  g >= b * 1.0000000000000001 and not self.color_is_yellow(port) and not self.color_is_blue(port) and g > r * 1.00000002:
            return True

        return False or color_sensor.color(port) == color.GREEN
        
    
    def color_is_red(self, port = None):

        """ 
        Checks if the color sensor on the given port detects red.
        Args:
            port: the port of the color sensor to check (default: forward color sensor)
        """

        port = port or self.forward_color_port

        return color_sensor.color(port) == color.RED


    def color_is_silver(self, port = None, silver_threshold = None):

        """
        Checks if the color sensor on the given port detects silver based on the RGB values.
        Args:
            port: the port of the color sensor to check (default: forward color sensor)
            silver_threshold: the threshold for detecting silver surfaces (default: silver_threshold)
        """

        port = port or self.forward_color_port
        silver_threshold = silver_threshold or self.silver_threshold
        r, g, b, intensity = color_sensor.rgbi(port)

        #checks if everything is greater than 1000
        if r > silver_threshold and g > silver_threshold and b > silver_threshold and intensity > 1010:
            #print("silver detected")
            return True
        else:
            return False
        
    
    def color_is_yellow(self, port = None):
        """
        Checks if the color sensor on the given port detects yellow based on the RGB values.
        """
        port = port or self.forward_color_port
        return color_sensor.color(port) == color.YELLOW
    
    def color_is_blue(self, port = None):
        """
        Checks if the color sensor on the given port detects blue based on the RGB values.
        """
        port = port or self.forward_color_port
        return color_sensor.color(port) == color.BLUE

    def reflection(self, port = None):

        """
        Gets the reflection value from the color sensor on the given port.
        Args:
            port: the port of the color sensor to check (default: forward color sensor)
        """

        port = port or self.forward_color_port
        return color_sensor.reflection(port)


    #---------------------------------------------------------
    #
    # HELPER FUNCTIONS FOR MOTORS
    #
    #---------------------------------------------------------


    def stop_motors(self):

        """
        Stops the given motors. If no motors are given, stops both motors left and right.
        Args:
            motors: the motors to stop (default: [left_motor_port, right_motor_port])
        """

        motor.stop(self.left_motor_port)
        motor.stop(self.right_motor_port)


    def set_motors_straight_forward(self, velocity = None):

        """
        Sets the motors to drive straight ahead using the motor module

        Args:
            velocity (int): Motor speed in degrees per second (default: SPEED_STRAIGHT_FORWARD)
        """
        
        velocity = velocity or self.speed_straight_forward
        motor.run(self.left_motor_port, -int(velocity))
        motor.run(self.right_motor_port, int(velocity))


    def set_motors_turn_right(self, high_velocity = None, low_velocity = None):

        """
        Sets the motors to a right turn using the motor module

        Args:
            high_velocity (int): Motor speed in degrees per second (default: speed_turn_high)
            low_velocity (int): Motor speed in degrees per second (default: speed_turn_low)
        """

        high_velocity = high_velocity or self.speed_turn_high
        low_velocity = low_velocity or self.speed_turn_low
        motor.run(self.left_motor_port, -(high_velocity))
        motor.run(self.right_motor_port, -(low_velocity))


    def set_motors_turn_left(self, high_velocity = None, low_velocity = None):

        """
        Sets the motors to a left turn using the motor module

        Args:
            high_velocity (int): Motor speed in degrees per second (default: speed_turn_high)
            low_velocity (int): Motor speed in degrees per second (default: speed_turn_low)
        """

        high_velocity = high_velocity or self.speed_turn_high
        low_velocity = low_velocity or self.speed_turn_low
        motor.run(self.left_motor_port, low_velocity)
        motor.run(self.right_motor_port, high_velocity)



    #---------------------------------------------------------
    #
    # HIGH LEVEL MOVEMENT FUNCTIONS
    # 
    #---------------------------------------------------------


    async def drive_straight(self, distance_cm: float,
                             *,
                            velocity = None,
                            stop_at_black = None,
                            ignore_silver = None,
                            wheel_diameter_cm = None,
                            kp = None):
        
        """
        Drive straight for a given distance (cm) and speed (°/s),
        using gyro-based correction.

        Returns True when stopped by black line

        Args:
            distance_cm (float): Distance to travel in cm.
            velocity (int): Motor speed in degrees per second.
            stop_at_black (bool): Defines if the function should stop when detecting a black line
            stop_at_silver (bool): Defines if the function should stop when detecting a silver surface (default: true)
            wheel_diameter_cm (float): Wheel diameter in cm.
            kp (float): Proportional gain for gyro correction (default 2.0).
        """

        velocity = velocity or self.speed_straight_forward
        wheel_diameter_cm = wheel_diameter_cm or self.wheel_diameter
        kp = kp or 2.0

        # Reset the gyro
        motion_sensor.reset_yaw(0)

        # Reset motor degrees
        motor.reset_relative_position(self.left_motor_port, 0)

        target_degrees = self.deg_for_distance(abs(distance_cm), wheel_diameter_cm)
        direction_forward_multiplicator = distance_cm * velocity
        if (direction_forward_multiplicator > 0):
            direction_forward_multiplicator = 1
        else:
            direction_forward_multiplicator = -1

        moved_degrees = 0

        while moved_degrees < target_degrees:
            if (self.color_is_silver() or self.color_is_silver(self.right_color_port) or self.color_is_silver(self.left_color_port)) and ignore_silver:
                motor_pair.stop(motor_pair.PAIR_1)
                return False
            if stop_at_black:
                if self.color_is_black() or self.color_is_black(self.right_color_port) or self.color_is_black(self.left_color_port) or self.color_is_silver():
                    motor_pair.stop(motor_pair.PAIR_1)
                    return True
            # SPIKE 3: tilt_angles()[0] gives yaw in deci-degrees with inverted sign
            yaw_deg = motion_sensor.tilt_angles()[0] * -0.1
            error = 0 - (yaw_deg * direction_forward_multiplicator)
            steer = int(max(-100, min(100, kp * error)))# Clamp steering between -100 and 100

            # move forward an set new steering correction
            motor_pair.move(motor_pair.PAIR_1, steer, velocity=abs(velocity)*direction_forward_multiplicator)
            moved_degrees = abs(motor.relative_position(self.left_motor_port))

        motor_pair.stop(motor_pair.PAIR_1)
        return False


    async def rotate_degrees(self, rotate_degrees: float,*, velocity = None, stop_at_black = None):

        """
        Turns the robot for a several degrees.
        Args:
            degrees: amount of degrees to turn the robot (use negative values to turn right and positiv values to turn left)
            stop_at_black: Defines if the function should stop when detecting a black line
        """

        velocity = velocity or self.speed_straight_forward
        stop_at_black = stop_at_black or False
        # Reset the gyro
        motion_sensor.reset_yaw(0)
        steer = -1
        if (rotate_degrees < 0):
            steer = -steer
        rotated_degrees = 0
        previous_yaw = motion_sensor.tilt_angles()[0] * 0.1

        while abs(rotate_degrees) > rotated_degrees:
            if stop_at_black:
                if self.color_is_black():
                    motor_pair.stop(motor_pair.PAIR_1)
                    return True
            # change velocity with percentage of rotated degrees, to slow down when getting closer to the target angle
            if steer < 0:
                self.set_motors_turn_left(high_velocity=velocity, low_velocity=velocity)
            else:
                self.set_motors_turn_right(high_velocity=velocity, low_velocity=velocity)
            current_yaw = motion_sensor.tilt_angles()[0] * 0.1
            yaw_delta = current_yaw - previous_yaw
            if yaw_delta > 180:
                yaw_delta -= 360
            elif yaw_delta < -180:
                yaw_delta += 360
            rotated_degrees += abs(yaw_delta)
            previous_yaw = current_yaw
        motor_pair.stop(motor_pair.PAIR_1)
        return False
    





#---------------------------------------------------------
#
# MAIN PROGRAM FUNCTIONS
# 
#---------------------------------------------------------






# global variables
last_color_left = "white"
last_color_right = "white"
previous_reading = 1000 # used for zone handling
white_black_distinguishing = 250 # for stricter handling

# initialization of the hub with all the ports and constants
hub = Hub(
    forward_color_port= port.A, # port for the forward facing color sensor, used for line following
    left_color_port= port.D, # port for the left facing color sensor, used for line following
    right_color_port= port.C,   # port for the right facing color sensor, used for line following
    left_motor_port= port.F, # port for the left motor, used for driving and turning
    right_motor_port= port.E, # port for the right motor, used for driving and turning
    forward_distance_port= port.B, # port for the forward facing distance sensor, used for obstacle detection
    obstacle_distance= 120, # how close an obstacle has to be to be detected, in mm
    wheel_diameter= 5.2, # in cm
    default_reflection_treshold= 65, # value between the reflection values of black and white surfaces, used for distinguishing between them
    silver_threshold= 600, # threshold for detecting silver surfaces, used for zone handling
    speed_straight_forward= 300, # speed for driving straight forward
    speed_turn_high= 340, # speed for the faster motor when turning
    speed_turn_low= 120, # speed for the slower motor when turning
    speed_slow= 160 # speed for slow movements, e.g. for some turns or for driving in the zone
)


def update_last_colors():

    """ Saves the last colors the sensors saw in order to detect changes and turns"""

    global last_color_right
    global last_color_left
    #save the last colors the right sensor sees
    if (hub.color_is_black(hub.right_color_port, white_black_distinguishing) and (last_color_right != "black")): 
        last_color_right = "black"
    if (hub.color_is_white(hub.right_color_port, white_black_distinguishing) and (last_color_right != "white")):
        last_color_right = "white"
    if (last_color_right != "green") and hub.color_is_green(hub.right_color_port) and last_color_right != "green":
        print("saved green")
        last_color_right = "green"

    #save the last color the left sensor sees
    if (hub.color_is_black(hub.left_color_port, white_black_distinguishing) and (last_color_left != "black")): 
        last_color_left = "black"
    if (hub.color_is_white(hub.left_color_port, white_black_distinguishing) and (last_color_left != "white")):
        last_color_left = "white"
    if ((last_color_left != "green") and hub.color_is_green(hub.left_color_port) and last_color_right != "green"):
        print("saved green")
        last_color_left = "green"


async def check_for_obstacles():

    """ Checks for obstacles in front of the robot and tries to drive around them by checking for free paths on the sides and going there,
      if there is no free path it tries to go back and turn and check again until it finds a way around the obstacle or gives up after 3 tries"""
    
    tries = 3

    if hub.distance_in_mm() < hub.obstacle_distance and hub.distance_in_mm() != -1:
        #wait for a short time to make sure its not a false positive
        sleep(0.2)
        if hub.distance_in_mm() < hub.obstacle_distance and hub.distance_in_mm() != -1:
            print("obstacle detected")
            hub.show_image("square")
            await hub.rotate_degrees(-90, velocity=hub.speed_slow)
            await hub.drive_straight(25, velocity=hub.speed_slow)
            await hub.rotate_degrees(90, velocity=hub.speed_slow)
            await hub.drive_straight(45, velocity=hub.speed_slow)
            await hub.rotate_degrees(90, velocity=hub.speed_slow)
            while True:
                if await hub.drive_straight(45, velocity=hub.speed_slow, stop_at_black=True):
                    break
                await hub.rotate_degrees(90, velocity=hub.speed_slow)
                tries -= 1
                if tries == 0:
                    hub.set_motors_straight_forward()
                    break
            await hub.drive_straight(5)
            await hub.rotate_degrees(-20)
            await hub.rotate_degrees(-80, stop_at_black=True)
            hub.show_image("arrow_front")


async def check_for_turns():

    """
    Checks for any green markings on the ground and lets the robot turn in the right direction if they follow up to white

    Attention: Requires the updateLastColor() function to be run in immediate advance in order to work properly
    """

    global last_color_left
    global last_color_right
    global white_black_distinguishing
    #check colors on the ground in case there is a u turn
    if hub.color_is_green(hub.left_color_port) and hub.color_is_green(hub.right_color_port):
        print("did a full turn")
        hub.show_image("arrow_back")
        last_color_right = "white"
        last_color_left = "white"
        await hub.rotate_degrees(180)
        hub.set_motors_straight_forward()
        sleep(0.2)
        hub.set_motors_turn_right()
        hub.show_image("arrow_front")

    #turn right if black follows to green
    if hub.color_is_green(hub.right_color_port) and last_color_right == "green":
            print("green")
            await hub.drive_straight(2)
            await hub.rotate_degrees(-12)
            if hub.color_is_black(hub.right_color_port):
                hub.show_image("arrow_right")
                await hub.rotate_degrees(-60)
                await hub.rotate_degrees(-40, stop_at_black=True)
                await hub.drive_straight(5)
                last_color_right = "black"
                hub.show_image("arrow_front")
            else:
                await hub.rotate_degrees(12)
    #turn right if black follows to green
    if hub.color_is_black(hub.left_color_port) and last_color_left == "green":
            print("green")
            await hub.drive_straight(2)
            await hub.rotate_degrees(12)
            if hub.color_is_black(hub.left_color_port):
                hub.show_image("arrow_left")
                print("turned left")
                print(hub.reflection(hub.left_color_port))
                await hub.rotate_degrees(60)
                await hub.rotate_degrees(40, stop_at_black=True)
                await hub.drive_straight(5)
                last_color_left = "black"
                hub.show_image("arrow_front")
            else:
                await hub.rotate_degrees(-12)

async def correct_line_path():

    """
    Corrects the current path of the robot to continue following the black line

    Also handles the end of the course
    """

    

        
    if (hub.color_is_black(hub.forward_color_port)):# and hub.color_is_white(hub.right_color_port) and hub.color_is_white(hub.left_color_port):
        hub.set_motors_straight_forward()
    else:
        #turn left if left color is black
        if (hub.color_is_black(hub.left_color_port)):
            hub.set_motors_turn_left()
        #turn right if right color is black
        if (hub.color_is_black(hub.right_color_port)):
            hub.set_motors_turn_right()
        #drive forward to cross the goal line and the quit the program if forward color is red
        if hub.color_is_red():
            #sleep for a short time to make sure its not a false positive
            sleep(0.2)
            if hub.color_is_red():
                await hub.drive_straight(2)
                raise SystemExit("Goal reached")
    #if everyting white, drive back and forth for checking line status in case of brake or lost line
    if (hub.color_is_white() and hub.color_is_white(hub.right_color_port) and hub.color_is_white(hub.left_color_port)):
        sleep(0.1)
        if (hub.color_is_white() and hub.color_is_white(hub.right_color_port) and hub.color_is_white(hub.left_color_port)):
            await hub.drive_straight(-5)
            if (hub.color_is_black(hub.forward_color_port) and hub.color_is_white(hub.right_color_port) and hub.color_is_white(hub.left_color_port)):
                await hub.drive_straight(8)
                await hub.drive_straight(30, stop_at_black=True)
                hub.show_image("arrow_front")
        else:
            hub.set_motors_straight_forward()


# last color checking for yellow and blue
default_speed = hub.speed_straight_forward
default_turn_high = hub.speed_turn_high
default_turn_low = hub.speed_turn_low
speed = "1" # 1 default ,0 slow ,2 fast
last_detected = "none"
since_last_detected = 1000000

def check_colors_for_markers():
    global since_last_detected
    global last_detected
    since_last_detected += 1
    if since_last_detected > 450:
        if hub.color_is_yellow(hub.right_color_port) or hub.color_is_yellow(hub.left_color_port):
            since_last_detected = 0
            print("yellow")
            if last_detected != "yellow":
                last_detected = "yellow"
                hub.speed_straight_forward = int(default_speed / 2)
                hub.speed_turn_high = int(default_turn_high / 2)
                hub.speed_turn_low = int(default_turn_low / 2)
            elif last_detected == "yellow":
                last_detected = "none"
                hub.speed_straight_forward = int(default_speed)
                hub.speed_turn_high = int(default_turn_high)
                hub.speed_turn_low = int(default_turn_low)
        if hub.color_is_blue(hub.right_color_port) or hub.color_is_blue(hub.left_color_port):
            print("blue")
            since_last_detected = 0
            if last_detected != "blue":
                last_detected = "blue"
                hub.speed_straight_forward = int(default_speed * 2)
                hub.speed_turn_high = int(default_turn_high * 2)
                hub.speed_turn_low = int(default_turn_low * 2)
            elif last_detected == "blue":
                last_detected = "none"
                hub.speed_straight_forward = int(default_speed)
                hub.speed_turn_high = int(default_turn_high)
                hub.speed_turn_low = int(default_turn_low)


async def main():
    """Main function"""
    hub.show_image("arrow_front")
    hub.set_motors_straight_forward()
    while True:
        check_colors_for_markers()
        await check_for_turns()
        await check_for_obstacles()
        await correct_line_path()
        update_last_colors()
runloop.run(main())
