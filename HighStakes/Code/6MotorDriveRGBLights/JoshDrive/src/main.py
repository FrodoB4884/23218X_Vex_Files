# --------------------------------------------------
# VEXcode Project Configuration and Setup
# --------------------------------------------------
from vex import *
import urandom  #type: ignore

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

# Controller Setup
controller_1 = Controller(PRIMARY)

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

#region Controller Configuration
# --------------------------------------------------
# Driver Control Input Monitoring Task
# --------------------------------------------------

# define variables used for controlling motors based on controller inputs
drivetrain_l_needs_to_be_stopped_controller_1 = False
drivetrain_r_needs_to_be_stopped_controller_1 = False

# define a task that will handle monitoring inputs from controller_1
def rc_auto_loop_function_controller_1():
    global drivetrain_l_needs_to_be_stopped_controller_1, drivetrain_r_needs_to_be_stopped_controller_1, remote_control_code_enabled
    # process the controller input every 20 milliseconds
    # update the motors based on the input values
    while True:
        if remote_control_code_enabled:
            # stop the motors if the brain is calibrating
            if inertial_sensor.is_calibrating():
                left_drive_smart.stop()
                right_drive_smart.stop()
                while inertial_sensor.is_calibrating():
                    sleep(25, MSEC)
            
            # calculate the drivetrain motor velocities from the controller joystick axies
            # left = axis3
            # right = axis2
            drivetrain_left_side_speed = controller_1.axis3.position()
            drivetrain_right_side_speed = controller_1.axis2.position()
            
            # check if the value is inside of the deadband range
            if drivetrain_left_side_speed < 5 and drivetrain_left_side_speed > -5:
                # check if the left motor has already been stopped
                if drivetrain_l_needs_to_be_stopped_controller_1:
                    # stop the left drive motor
                    left_drive_smart.stop()
                    # tell the code that the left motor has been stopped
                    drivetrain_l_needs_to_be_stopped_controller_1 = False
            else:
                # reset the toggle so that the deadband code knows to stop the left motor next
                # time the input is in the deadband range
                drivetrain_l_needs_to_be_stopped_controller_1 = True
            # check if the value is inside of the deadband range
            if drivetrain_right_side_speed < 5 and drivetrain_right_side_speed > -5:
                # check if the right motor has already been stopped
                if drivetrain_r_needs_to_be_stopped_controller_1:
                    # stop the right drive motor
                    right_drive_smart.stop()
                    # tell the code that the right motor has been stopped
                    drivetrain_r_needs_to_be_stopped_controller_1 = False
            else:
                # reset the toggle so that the deadband code knows to stop the right motor next
                # time the input is in the deadband range
                drivetrain_r_needs_to_be_stopped_controller_1 = True
            
            # only tell the left drive motor to spin if the values are not in the deadband range
            if drivetrain_l_needs_to_be_stopped_controller_1:
                left_drive_smart.set_velocity(drivetrain_left_side_speed, PERCENT)
                left_drive_smart.spin(FORWARD)
            # only tell the right drive motor to spin if the values are not in the deadband range
            if drivetrain_r_needs_to_be_stopped_controller_1:
                right_drive_smart.set_velocity(drivetrain_right_side_speed, PERCENT)
                right_drive_smart.spin(FORWARD)
        # wait before repeating the process
        wait(20, MSEC)

# define variable for remote controller enable/disable
remote_control_code_enabled = True

rc_auto_loop_thread_controller_1 = Thread(rc_auto_loop_function_controller_1)

#endregion Controller Configuration

# |-------------------------------------------------|
# |                                                 |
# |	Project:      JoshDrive                         |
# |	Team:         23218X                            |
# |	Description:  Josh's Drivetrain configuration   |
# |                                                 |
# |-------------------------------------------------|

#region Variable Initialization
# ----------------
# Global variables
# ----------------

conveyor_direction = 0
loop_count = 0

mogomech_state = False
paddle_state = False

drivetrain_stopping_type = COAST

x, y = 0, 0
# Previous encoder values
prev_left_encoder = 0.0
prev_right_encoder = 0.0

pre_auton_done = False

# Constants
GEAR_RATIO = 1        # Motor to wheel gear ratio

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

#region Autonomous Routines
# -------------------
# Autonomous Routines
# -------------------
def red_left():
    global x, y
    brain.screen.print_at("Red Left", x=100, y=120)
    color_red()

    x, y = -1500, 600
    inertial_sensor.set_heading(90)
    # Get mogo and intake preload
    drivetrain.turn_to_heading(direction_of(-1200, 346))
    drivetrain.drive_for(FORWARD, distance_to(-1200, 346), DistanceUnits.MM)
    wait(1, SECONDS)
    rear_to_goal = (direction_of(-600, 600) - 180) % 360
    drivetrain.turn_to_heading(rear_to_goal)

    drivetrain.drive(REVERSE)
    while limit_sensor.pressing() == False and x < -600:
        wait(20, MSEC)
    set_mogomech_state(False)
    drivetrain.stop()
    drivetrain.turn_to_heading(0)
    wait(1, SECONDS)
    set_intake_state(1)
    # Get one other ring
    drivetrain.turn_to_heading(direction_of(-600, 1200))
    drivetrain.drive_for(FORWARD, distance_to(-600, 1200), DistanceUnits.MM)
    wait(1, SECONDS)
    # Stop intake and go to touch ladder
    set_intake_state(0)
    drivetrain.turn_to_heading(direction_of(-600, 0))
    drivetrain.drive_for(FORWARD, distance_to(-600, 0) - 240, DistanceUnits.MM)

def red_right():
    global x, y
    brain.screen.print_at("Red Right", x=100, y=120)
    color_red()

    x, y = -1500, -600
    inertial_sensor.set_heading(90)
    # Get mogo and intake preload
    drivetrain.turn_to_heading(direction_of(-1200, -346))
    drivetrain.drive_for(FORWARD, distance_to(-1200, -346), DistanceUnits.MM)
    wait(1, SECONDS)
    rear_to_goal = (direction_of(-600, -600) - 180) % 360
    drivetrain.turn_to_heading(rear_to_goal)

    drivetrain.drive(REVERSE)
    while limit_sensor.pressing() == False and x < -600:
        wait(20, MSEC)
    set_mogomech_state(False)
    drivetrain.stop()
    drivetrain.turn_to_heading(180)
    wait(1, SECONDS)
    set_intake_state(1)
    # Get one other ring
    drivetrain.turn_to_heading(direction_of(-600, -1200))
    drivetrain.drive_for(FORWARD, distance_to(-600, -1200), DistanceUnits.MM)
    wait(1, SECONDS)
    # Stop intake and go to touch ladder
    set_intake_state(0)
    drivetrain.turn_to_heading(direction_of(-600, 0))
    drivetrain.drive_for(FORWARD, distance_to(-600, 0) - 240, DistanceUnits.MM)

def blue_right():
    global x, y
    brain.screen.print_at("Blue Right", x=100, y=120)
    color_blue()
    
    x, y = 1500, 600
    inertial_sensor.set_heading(270)
    # Get mogo and intake preload
    drivetrain.turn_to_heading(direction_of(1200, 346))
    drivetrain.drive_for(FORWARD, distance_to(1200, 346), DistanceUnits.MM)
    wait(1, SECONDS)
    rear_to_goal = (direction_of(600, 600) - 180) % 360
    drivetrain.turn_to_heading(rear_to_goal)

    drivetrain.drive(REVERSE)
    while limit_sensor.pressing() == False and x > 600:
        wait(20, MSEC)
    set_mogomech_state(False)
    drivetrain.stop()
    drivetrain.turn_to_heading(0)
    wait(1, SECONDS)
    set_intake_state(1)
    # Get one other ring
    drivetrain.turn_to_heading(direction_of(600, 1200))
    drivetrain.drive_for(FORWARD, distance_to(600, 1200), DistanceUnits.MM)
    wait(1, SECONDS)
    # Stop intake and go to touch ladder
    set_intake_state(0)
    drivetrain.turn_to_heading(direction_of(600, 0))
    drivetrain.drive_for(FORWARD, distance_to(600, 0) - 240, DistanceUnits.MM)

def blue_left():
    global x, y
    brain.screen.print_at("Blue Left", x=100, y=120)
    color_blue()

    x, y = 1500, -600
    inertial_sensor.set_heading(270)
    # Get mogo and intake preload
    drivetrain.turn_to_heading(direction_of(1200, -346))
    drivetrain.drive_for(FORWARD, distance_to(1200, -346), DistanceUnits.MM)
    wait(1, SECONDS)
    rear_to_goal = (direction_of(600, -600) - 180) % 360
    drivetrain.turn_to_heading(rear_to_goal)

    drivetrain.drive(REVERSE)
    while limit_sensor.pressing() == False and x > 600:
        wait(20, MSEC)
    set_mogomech_state(False)
    drivetrain.stop()
    drivetrain.turn_to_heading(180)
    wait(1, SECONDS)
    set_intake_state(1)
    # Get one other ring
    drivetrain.turn_to_heading(direction_of(600, -1200))
    drivetrain.drive_for(FORWARD, distance_to(600, -1200), DistanceUnits.MM)
    wait(1, SECONDS)
    # Stop intake and go to touch ladder
    set_intake_state(0)
    drivetrain.turn_to_heading(direction_of(600, 0))
    drivetrain.drive_for(FORWARD, distance_to(600, 0) - 240, DistanceUnits.MM)

#endregion Autonomous Routines

#region Brain Buttons
# -------------
# Brain Buttons
# -------------

# Create holder for chosen function
auton_function = None

# Initialise buttons and corresponding colors + functions for the pre-autonomous
button_rl = {"x": 20, "y": 50, "width": 80, "height": 40, "label": "Red L", "color": Color.RED, "function": red_left}
button_br = {"x": 120, "y": 50, "width": 80, "height": 40, "label": "Blue R", "color": Color.BLUE, "function": blue_right}
button_rr = {"x": 20, "y": 130, "width": 80, "height": 40, "label": "Red R", "color": Color.RED, "function": red_right}
button_bl = {"x": 120, "y": 130, "width": 80, "height": 40, "label": "Blue L", "color": Color.BLUE, "function": blue_left}

buttons = [button_rl, button_br, button_rr, button_bl]

def draw_buttons():
    # Draw all buttons on the screen.
    brain.screen.clear_screen()
    for button in buttons:
        brain.screen.draw_rectangle(button["x"], button["y"], button["width"], button["height"], button["color"])
        brain.screen.print_at(button["label"], x = button["x"] + 10, y = button["y"] + 20)

def check_button_pressed(x, y):
    # Check if a button is pressed based on the touch coordinates.
    for button in buttons:
        if (button["x"] <= x <= button["x"] + button["width"] and
                button["y"] <= y <= button["y"] + button["height"]):
            return button
    return None

#endregion Brain Buttons

#region Pre Autonomous Code
# -------------------
# Pre Autonomous Code
# -------------------
def pre_autonomous():
    global pre_auton_done, auton_function, conveyor_direction
    brain.screen.clear_screen()
    
    brain.screen.print("Initializing")

    inertial_sensor.calibrate()

    drivetrain.set_stopping(COAST)

    conveyor.set_velocity(100, PERCENT)
    intake.set_velocity(100, PERCENT)
    drivetrain.set_drive_velocity(50, PERCENT)
    drivetrain.set_turn_velocity(50, PERCENT)

    conveyor_direction = 0
    set_mogomech_state(True)
    set_paddle_state(False)
    
    while inertial_sensor.is_calibrating():
        wait(25, MSEC)

    draw_buttons()
    
    while auton_function == None:
        if brain.screen.pressing():
            touch_x = brain.screen.x_position()
            touch_y = brain.screen.y_position()

            pressed_button = check_button_pressed(touch_x, touch_y)
            if pressed_button:
                brain.screen.clear_screen()
                auton_function = pressed_button["function"]

    pre_auton_done = True

#endregion Pre Autonomous Code

#region Autonomous Code
# ---------------
# Autonomous Code
# ---------------
def autonomous():
    while pre_auton_done == False:
        wait(20,MSEC)
    brain.screen.clear_screen()
    brain.screen.print("Autonomous Code")
    auton_function()

#endregion Autonomous Code

#region Driver Control
# --------------
# Driver Control
# --------------
def driver_control():
    brain.screen.clear_screen()
        
    drivetrain.set_drive_velocity(100, PERCENT)
    drivetrain.set_turn_velocity(100, PERCENT)


    controller_1.rumble("..-")
    controller_1.screen.clear_screen
    controller_1.screen.print("MECHANUS  OPTIMUS")

    # Set controller keybinds
    controller_1.buttonR1.pressed(intake_toggle, [-1])
    controller_1.buttonR2.pressed(intake_toggle, [1])

    controller_1.buttonL1.pressed(mogomech_toggle)
    controller_1.buttonL2.pressed(mogomech_toggle)

    controller_1.buttonA.pressed(paddle_toggle)

    controller_1.buttonB.pressed(toggle_drive_hold)

    controller_1.buttonLeft.pressed(color_blue)
    controller_1.buttonRight.pressed(color_red)
    controller_1.buttonUp.pressed(color_purple)
    controller_1.buttonDown.pressed(color_off)

    controller_1.buttonY.pressed(rainbow_loop)
    
    # ------------------
    # Robot Data Display
    # ------------------
    while True:
        vision_objects = ai_vision_sensor.take_snapshot(AiVision.ALL_AIOBJS)
        brain.screen.clear_screen()
        brain.screen.set_cursor(1,1)
        #brain.screen.print(vision_objects)
        brain.screen.print("Objects: ")
        for obj in vision_objects:
            if obj.id == GameElements.MOBILE_GOAL:
                brain.screen.print("Mogo")
            elif obj.id == GameElements.RED_RING:
                brain.screen.print("Red")
            elif obj.id == GameElements.BLUE_RING:
                brain.screen.print("Blue")
            else:
                brain.screen.print("?")
            brain.screen.print("|")

        # Print if mogo is engaged
        controller_1.screen.clear_line(1)
        controller_1.screen.set_cursor(1,1)
        controller_1.screen.print("Mogomech: ")
        if mogomech_state == True:
            controller_1.screen.print("UP")
        else:
            controller_1.screen.print("DOWN")

        # Print temperature values on controller
        controller_1.screen.clear_line(2)
        controller_1.screen.set_cursor(2,1)
        controller_1.screen.print("Drivetrain: ")
        if drivetrain.temperature() < 55:
            controller_1.screen.print(str(drivetrain.temperature()))
        else:
            controller_1.screen.print("HIGH")
        if drivetrain_stopping_type == COAST:
            controller_1.screen.print(" COAST")
        elif drivetrain_stopping_type == HOLD:
            controller_1.screen.print(" HOLD")

        controller_1.screen.clear_line(3)
        controller_1.screen.set_cursor(3,1)
        controller_1.screen.print("Intake: ")
        if intake.temperature() < 55:
            controller_1.screen.print(str(intake.temperature()))
        else:
            controller_1.screen.print("HIGH")

        if conveyor_is_stuck():
            wait(20, MSEC)
            if conveyor_is_stuck():
                controller_1.rumble("---")
                set_intake_state(-1)
                while conveyor_is_stuck() == True:
                    wait(50, MSEC)
                set_intake_state(0)

        wait(20, MSEC)

#endregion Driver Control

#region Mechanisms Functions
# -------------------------------------------
# Functions for Mechanisms in the Robot
# -------------------------------------------

# toggles
def intake_toggle(direction):
    global conveyor_direction
    if conveyor_direction == 0:
        set_intake_state(direction)
    else:
        set_intake_state(0)

def mogomech_toggle():
    global mogomech_state 
    # if true becomes false and if false becomes true
    new_state = (mogomech_state == False)
    set_mogomech_state(new_state)

def paddle_toggle():
    global paddle_state
    new_state = (paddle_state == False)
    set_paddle_state(new_state)

def toggle_drive_hold():
    global drivetrain_stopping_type
    if drivetrain_stopping_type == HOLD:
        drivetrain.set_stopping(COAST)
        drivetrain_stopping_type = COAST
    else:
        drivetrain.set_stopping(HOLD)
        drivetrain_stopping_type = HOLD


# state setters
def set_intake_state(direction):
    global conveyor_direction 
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
    global mogomech_state
    mogomech_state = state
    mogomech.set(state)

def set_paddle_state(state):
    global paddle_state
    paddle_state = state
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
    while controller_1.buttonY.pressing():
        rainbow(loop_count % 60)
        wait(0.1, SECONDS)
        loop_count += 1

#endregion LED Lights

# ----------------------------
# Creates Competition Instance
# ----------------------------

comp = Competition(driver_control, autonomous)
pre_autonomous()