# --------------------------------------------------
# VEXcode Project Configuration and Setup
# --------------------------------------------------
from vex import *
import urandom  #type: ignore
import math

#region Robot Configuration Setup
# Brain should be defined by default
brain = Brain()

# -----------------
# Drivetrain Setup
# -----------------
# Left drivetrain motors (ports 11, 12, 13)
left_motor_a = Motor(Ports.PORT11, GearSetting.RATIO_18_1, True)
left_motor_b = Motor(Ports.PORT12, GearSetting.RATIO_18_1, True)
left_motor_c = Motor(Ports.PORT13, GearSetting.RATIO_18_1, False)
left_drive_smart = MotorGroup(left_motor_a, left_motor_b, left_motor_c)

# Right drivetrain motors (ports 18, 19, 20)
right_motor_a = Motor(Ports.PORT18, GearSetting.RATIO_18_1, False)
right_motor_b = Motor(Ports.PORT19, GearSetting.RATIO_18_1, False)
right_motor_c = Motor(Ports.PORT20, GearSetting.RATIO_18_1, True)
right_drive_smart = MotorGroup(right_motor_a, right_motor_b, right_motor_c)

# Inertial Sensor (port 15)
inertial_sensor = Inertial(Ports.PORT15)

# drivetrain constants 
WHEEL_DIAMETER = 100  # mm
WHEEL_CIRCUMFERENCE = math.pi * WHEEL_DIAMETER
TRACK_WIDTH = 355.6   # mm

# Complete drivetrain configuration
drivetrain = SmartDrive(left_drive_smart, right_drive_smart, inertial_sensor, WHEEL_CIRCUMFERENCE, TRACK_WIDTH, 205, MM, 1)

# -------------------
# Mechanisms Setup
# -------------------
conveyor = Motor(Ports.PORT3, GearSetting.RATIO_6_1, True)
intake = Motor(Ports.PORT4, GearSetting.RATIO_18_1, True)

paddle = DigitalOut(brain.three_wire_port.b)
mogomech = DigitalOut(brain.three_wire_port.a)

# Addressable LED setup (port C)
addled = AddressableLed(brain.three_wire_port.c)

# Switch Sensor
limit_sensor = Limit(brain.three_wire_port.d)

# ------------------
# AI Vision Sensor
# ------------------

# AI Classification Competition Element IDs
class GameElements:
    MOBILE_GOAL = 0
    RED_RING = 1
    BLUE_RING = 2

ai_vision_sensor = AiVision(Ports.PORT14, AiVision.ALL_AIOBJS)

#endregion Robot Configuration Setup

#region Other Configurations
# --------------------------------------------------
# Utility and Helper Functions
# --------------------------------------------------

def initializeRandomSeed():
    #Initializes a random seed for urandom using battery and timer values.
    wait(100, MSEC)
    random = brain.battery.voltage(MV) + brain.battery.current(CurrentUnits.AMP) * 100 + brain.timer.system_high_res()
    urandom.seed(int(random))

initializeRandomSeed()  # Call random seed initializer

def calibrate_drivetrain():
    # Calibrate the Drivetrain Inertial
    global vexcode_initial_drivetrain_calibration_completed
    sleep(200, MSEC)
    brain.screen.print("Calibrating")
    brain.screen.next_row()
    brain.screen.print("Inertial")
    inertial_sensor.calibrate()
    while inertial_sensor.is_calibrating():
        sleep(25, MSEC)
    vexcode_initial_drivetrain_calibration_completed = True
    brain.screen.clear_screen()
    brain.screen.set_cursor(1, 1)

def play_vexcode_sound(sound_name):
    # Helper to make playing sounds from the V5 in VEXcode easier and
    # keeps the code cleaner by making it clear what is happening.
    print("VEXPlaySound:" + sound_name)
    wait(5, MSEC)

# --------------------------------------------------
# Autonomous and Driver Control Pre-Setup
# --------------------------------------------------

remote_control_code_enabled = True
vexcode_initial_drivetrain_calibration_completed = False

calibrate_drivetrain()  # Drivetrain calibration
wait(200, MSEC)  # Ensure clean start
print("\033[2J")  # Clear console output

#endregion Other Configurations

# |-------------------------------------------------|
# |                                                 |
# | Project:      AutonomousSkills                  |
# | Team:         23218X                            |
# | Description:  Autonomous Program for            |
# |               competition skills                |
# |                                                 |
# |-------------------------------------------------|

#region Variable Initialization
# ----------------
# Global variables
# ----------------

conveyor_direction = 0
loop_count = 0

x, y =   -1500, -773

# Previous encoder values
prev_left_encoder = 0.0
prev_right_encoder = 0.0

pre_auton_done = False

# Constants
WHEEL_DIAMETER = 0.1  # meters (adjust to your wheel size)
TRACK_WIDTH = 0.35    # meters (distance between left and right wheels)
GEAR_RATIO = 1        # Motor to wheel gear ratio
WHEEL_CIRCUMFERENCE = math.pi * WHEEL_DIAMETER

CENTER_TO_GOAL_DISTANCE = 400

RED = [Color(0x800000)]*60
BLUE = [Color(0x0000ff)]*60
PURPLE = [Color(0x800080)]*60
RAINBOW = [
	Color(0xFF0000), Color(0xFF1900), Color(0xFF3200), Color(0xFF4C00), Color(0xFF6600),
	Color(0xFF7F00), Color(0xFF9900), Color(0xFFB300), Color(0xFFCC00), Color(0xFFE600),
	Color(0xFFFF00), Color(0xE6FF00), Color(0xCCFF00), Color(0xB3FF00), Color(0x99FF00),
	Color(0x7FFF00), Color(0x66FF00), Color(0x4CFF00), Color(0x32FF00), Color(0x19FF00),
	Color(0x00FF00), Color(0x00FF19), Color(0x00FF32), Color(0x00FF4C), Color(0x00FF66),
	Color(0x00FF7F), Color(0x00FF99), Color(0x00FFB3), Color(0x00FFCC), Color(0x00FFE6),
	Color(0x00FFFF), Color(0x00E6FF), Color(0x00CCFF), Color(0x00B3FF), Color(0x0099FF),
	Color(0x007FFF), Color(0x0066FF), Color(0x004CFF), Color(0x0032FF), Color(0x0019FF),
	Color(0x0000FF), Color(0x1900FF), Color(0x3200FF), Color(0x4C00FF), Color(0x6600FF),
	Color(0x7F00FF), Color(0x9900FF), Color(0xB300FF), Color(0xCC00FF), Color(0xE600FF),
	Color(0xFF00FF), Color(0xFF00E6), Color(0xFF00CC), Color(0xFF00B3), Color(0xFF0099),
	Color(0xFF007F), Color(0xFF0066), Color(0xFF004C), Color(0xFF0032), Color(0xFF0019)
]

#endregion Variable Initialization

#region Data Logging
# --------------------------------------
# Data Logging Function for Match Replay
# --------------------------------------

def data_logging():
    while True:
        brain.screen.clear_line(1)
        brain.screen.set_cursor(1,1)
        brain.screen.print(str(x)+" "+str(y)+" "+str(inertial_sensor.heading()))
        if brain.sdcard.is_inserted():
            pass
        wait(20, MSEC)

data_logging = Thread(data_logging) #type: ignore
#endregion Data Logging

#region Odometry
# --------
# Odometry
# --------

TURN_THRESHOLD = 9

def update_position():
    global x, y, prev_left_encoder, prev_right_encoder

    # Define scaling factor for calibration
    SCALING_FACTOR = 1  # Adjust this value based on testing

    # Read current motor encoder positions
    current_left_encoder = left_motor_a.position(DEGREES)
    current_right_encoder = right_motor_a.position(DEGREES)

    # Calculate change in encoder values
    delta_left_encoder = current_left_encoder - prev_left_encoder
    delta_right_encoder = current_right_encoder - prev_right_encoder

    # Update previous encoder values
    prev_left_encoder = current_left_encoder
    prev_right_encoder = current_right_encoder

    # Convert encoder changes to distance with scaling factor
    delta_left = delta_left_encoder * (WHEEL_CIRCUMFERENCE / 360) * SCALING_FACTOR  # mm
    delta_right = delta_right_encoder * (WHEEL_CIRCUMFERENCE / 360) * SCALING_FACTOR  # mm

    # Check for turning
    if abs(delta_left + delta_right) < TURN_THRESHOLD:  # Small sum implies turning
        return x, y  # Skip position update

    # Calculate robot motion
    delta_distance = (delta_left + delta_right) / 2

    # Use inertial sensor for angle
    theta = math.radians(inertial_sensor.heading())  # Convert heading to radians

    # Update global position
    x += delta_distance * math.sin(theta)
    y += delta_distance * math.cos(theta)

    return x, y


def background_updating():
    global x, y
    while True:
        x, y = update_position()
        wait(20, MSEC)

background_updating_thread = Thread(background_updating) #type: ignore

# input in mm
def direction_of(target_x, target_y):
    # Calculate the angle using atan2, aligning 0° to the positive Y-axis
    theta_target = math.degrees(math.atan2(target_x - x, target_y - y))  # Flip dx and dy

    # Normalize to [0, 360)
    heading = (theta_target + 360) % 360

    return heading

# input in mm
def distance_to(target_x, target_y):
    delta_x = target_x - x
    delta_y = target_y - y

    euclid = math.sqrt((delta_x**2)+(delta_y**2))

    distance = euclid

    return distance

#endregion Odometry

#region Pre Autonomous Code
# -------------------
# Pre Autonomous Code
# -------------------
def pre_autonomous():
    global pre_auton_done, conveyor_direction
    brain.screen.clear_screen()
    
    brain.screen.print("Initializing")

    inertial_sensor.calibrate()

    drivetrain.set_stopping(COAST)

    conveyor.set_velocity(100, PERCENT)
    intake.set_velocity(100, PERCENT)
    drivetrain.set_drive_velocity(100, PERCENT)
    drivetrain.set_turn_velocity(100, PERCENT)

    set_intake_state(0)
    set_mogomech_state(True)
    set_paddle_state(False)
    
    while inertial_sensor.is_calibrating():
        wait(20, MSEC)
        
    inertial_sensor.set_heading(240)
    pre_auton_done = True

#endregion Pre Autonomous Code

#region Autonomous Code
# ---------------
# Autonomous Code
# ---------------

def autonomous():
    while pre_auton_done == False:
        wait(20, MSEC)

    brain.screen.clear_screen()
    brain.screen.print("Autonomous Code")

    # ------------
    # Quadrant One
    # ------------
    # Clamp mobile goal
    drivetrain.drive(REVERSE)
    while limit_sensor.pressing() == False and x < -1000:
        wait(20, MSEC)
    set_mogomech_state(False)
    drivetrain.stop()
    # Score preload (first ring) and start intake for next section
    set_intake_state(1)
    # Go to rings in this quadrant
    #   Second ring
    drivetrain.turn_to_heading(direction_of(-600, -600))
    drivetrain.drive_for(FORWARD, distance_to(-600, -600), DistanceUnits.MM)
    #   Third ring
    drivetrain.turn_to_heading(direction_of(-600, -1200))
    drivetrain.drive_for(FORWARD, distance_to(-600, -1200), DistanceUnits.MM)
    #   Fourth ring
    drivetrain.turn_to_heading(direction_of(-1200, -1200))
    drivetrain.drive_for(FORWARD, distance_to(-1200, -1200), DistanceUnits.MM)
    #   Fifth ring
    drivetrain.turn_to_heading(direction_of(-1500, -1200))
    drivetrain.drive_for(FORWARD, distance_to(-1500, -1200), DistanceUnits.MM)
    wait(1, SECONDS)
    set_intake_state(0)
    # Turn so mogo is facing corner then place it there
    rear_to_corner = (direction_of(-1800, -1800) - 180) % 360
    drivetrain.turn_to_heading(rear_to_corner)
    drivetrain.drive_for(REVERSE, distance_to(-1800, -1800) - CENTER_TO_GOAL_DISTANCE, DistanceUnits.MM)
    set_mogomech_state(True)

    drivetrain.turn_to_heading(direction_of(-1200, 0))
    drivetrain.drive_for(FORWARD, distance_to(-1200, 0), DistanceUnits.MM)
    # ------------
    # Quadrant Two
    # ------------
    # Clamp mobile goal
    drivetrain.set_heading(180)
    drivetrain.drive(REVERSE)
    while limit_sensor.pressing() == False and y < 800:
        wait(20, MSEC)
    wait(0.1, SECONDS)
    set_mogomech_state(False)
    drivetrain.stop()
    # Start intake for next section
    set_intake_state(1)
    # Go to rings in this quadrant
    #   First ring
    drivetrain.turn_to_heading(direction_of(-600, 600))
    drivetrain.drive_for(FORWARD, distance_to(-600, 600), DistanceUnits.MM)
    #   Second ring
    drivetrain.turn_to_heading(direction_of(-600, 1200))
    drivetrain.drive_for(FORWARD, distance_to(-600, 1200), DistanceUnits.MM)
    #   Third ring
    drivetrain.turn_to_heading(direction_of(0, 1500))
    drivetrain.drive_for(FORWARD, distance_to(0, 1500) - CENTER_TO_GOAL_DISTANCE, DistanceUnits.MM)
    #   Fourth ring
    drivetrain.turn_to_heading(direction_of(-1200, 1200))
    drivetrain.drive_for(FORWARD, distance_to(-1200, 1200), DistanceUnits.MM)
    #   Fifth ring
    drivetrain.turn_to_heading(direction_of(-1500, 1200))
    drivetrain.drive_for(FORWARD, distance_to(-1500, 1200), DistanceUnits.MM)
    wait(1, SECONDS)
    set_intake_state(0)
    # Turn so mogo is facing corner then place it there
    rear_to_corner = (direction_of(-1800, 1800) - 180) % 360
    drivetrain.turn_to_heading(rear_to_corner)
    drivetrain.drive_for(REVERSE, distance_to(-1800, 1800) - CENTER_TO_GOAL_DISTANCE, DistanceUnits.MM)
    set_mogomech_state(True)

    # ----------
    # Final Half
    # ----------

    drivetrain.turn_to_heading(direction_of(1200, 750))
    drivetrain.drive_for(FORWARD, distance_to(1200, 750), DistanceUnits.MM)

    # Clamp mobile goal
    drivetrain.set_heading(0)
    drivetrain.drive(REVERSE)
    while limit_sensor.pressing() == False and y > -200:
        wait(20, MSEC)
    wait(0.1, SECONDS)
    set_mogomech_state(False)
    drivetrain.stop()

    drivetrain.set_heading(direction_of(600, -600))

    # Use vision sensor to find red rings
    target_center_x = 0
    target_center_y = 0

    while target_center_x > 170 and target_center_x < 150:
        vision_objects = ai_vision_sensor.take_snapshot(AiVision.ALL_AIOBJS)
        for object in vision_objects:
            # make target the closest red ring
            if object.id == GameElements.RED_RING:
                if target_center_y > object.centerY:
                    target_center_x = object.centerX
                    target_center_y = object.centerY
        if target_center_x != 0:
            # turn to face object
            if target_center_x > 170:
                drivetrain.turn_for(RIGHT, 10)
            if target_center_x < 150:
                drivetrain.turn_for(LEFT, 10)
        else:
            drivetrain.drive_for(FORWARD, 2)

    # Drive to collect ring
    set_intake_state(1)
    drivetrain.drive_for(FORWARD, 400, DistanceUnits.MM)
    wait(1, SECONDS)
    set_intake_state(0)

    # Turn so mogo is facing corner then place it there
    rear_to_corner = (direction_of(1800, -1800) - 180) % 360
    drivetrain.turn_to_heading(rear_to_corner)
    drivetrain.drive_for(REVERSE, distance_to(1800, -1800) - CENTER_TO_GOAL_DISTANCE, DistanceUnits.MM)
    set_mogomech_state(True)

        # ~ Fin ~: 33pts

#endregion Autonomous Code

#region Driver Control
# -----------------------------------------------
# Driver Control (Not used for autonomous script)
# -----------------------------------------------
def driver_control():
    pass

#endregion Driver Control

#region Mechanisms Functions
# -------------------------------------------
# Functions for Mechanisms in the Robot
# -------------------------------------------

def set_intake_state(direction):
    conveyor_direction = direction
    if direction == 1:
        conveyor.spin(FORWARD)
        intake.spin(FORWARD)
    elif direction == -1:
        conveyor.spin(REVERSE)
        intake.spin(REVERSE)
    else:
        conveyor.stop()
        intake.stop()

def set_mogomech_state(state):
    mogomech.set(state)

def set_paddle_state(state):
    paddle.set(state)

def conveyor_is_stuck():
    if conveyor_direction != 0:
        if abs(conveyor.velocity()) <= 0:
            return True
        else:
            return False
    else:
        return False

#endregion Other Mechanisms Functions

#region LED Lights
# ------------------------
# Functions for LED lights
# ------------------------
def color_purple():
    global PURPLE

    addled.set(PURPLE)

def color_blue():
    global BLUE

    addled.set(BLUE)

def color_red():
    global RED

    addled.set(RED)

def color_off():
    addled.clear()

def rainbow(offset):
    global RAINBOW

    n = len(RAINBOW)
    offset %= n  # Handle shifts larger than the list length
    display_rainbow = RAINBOW[-offset:] + RAINBOW[:-offset]

    addled.set(display_rainbow)

def rainbow_loop():
    global RAINBOW
    global loop_count
    rainbow(loop_count % 60)
    wait(0.1, SECONDS)
    loop_count += 1

#endregion LED Lights

# ----------------------------
# Creates Competition Instance
# ----------------------------

comp = Competition(driver_control, autonomous)
pre_autonomous()
