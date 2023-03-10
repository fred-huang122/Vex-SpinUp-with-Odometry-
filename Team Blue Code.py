# region VEXcode Generated Robot Configuration
from vex import *
import urandom

# Brain should be defined by default
brain = Brain()

# Robot configuration code
expander_13 = Triport(Ports.PORT13)
controller_1 = Controller(PRIMARY)
FLdrive = Motor(Ports.PORT12, GearSetting.RATIO_18_1, False)
FRdrive = Motor(Ports.PORT19, GearSetting.RATIO_18_1, True)
BLdrive = Motor(Ports.PORT11, GearSetting.RATIO_18_1, False)
BRdrive = Motor(Ports.PORT20, GearSetting.RATIO_18_1, True)
Rotationx = Motor(Ports.PORT18, GearSetting.RATIO_36_1, False)
EndgameA = DigitalOut(brain.three_wire_port.b)
EndgameB = DigitalOut(brain.three_wire_port.c)
Trigger = DigitalOut(brain.three_wire_port.a)
ShooterA = Motor(Ports.PORT9, GearSetting.RATIO_6_1, False)
ShooterB = Motor(Ports.PORT10, GearSetting.RATIO_6_1, True)
inertial_3 = Inertial(Ports.PORT3)
Roller = Motor(Ports.PORT4, GearSetting.RATIO_18_1, False)
Colorchecker = Optical(Ports.PORT7)
XaxisRotation = Rotation(Ports.PORT14, False)
Blocker = DigitalOut(brain.three_wire_port.d)
RightEncoder = Encoder(expander_13.e)
BackEncoder = Encoder(expander_13.c)
LeftEncoder = Encoder(expander_13.a)

# wait for rotation sensor to fully initialize
wait(30, MSEC)
# endregion VEXcode Generated Robot Configuration

vexcode_brain_precision = 0
vexcode_console_precision = 0
vexcode_controller_1_precision = 0
Shoot = 0
rpm_value = 0
Rollervar = 0
triggervar = 0
autoroller = 0
team_colour = 0
Endgame = False
Orginal_Colour_Blue = False


def Stop_Drivertrian():
    global Shoot, rpm_value, Rollervar, triggervar, autoroller, team_colour, Endgame, Orginal_Colour_Blue, vexcode_brain_precision, vexcode_console_precision, vexcode_controller_1_precision
    FLdrive.stop()
    FRdrive.stop()
    BLdrive.stop()
    BRdrive.stop()


def SetVelocity(SetVelocity_val, perc_or_rpm):
    global Shoot, rpm_value, Rollervar, triggervar, autoroller, team_colour, Endgame, Orginal_Colour_Blue, vexcode_brain_precision, vexcode_console_precision, vexcode_controller_1_precision
    if str(perc_or_rpm) == "rpm":
        FLdrive.set_velocity(SetVelocity_val, RPM)
        FRdrive.set_velocity(SetVelocity_val, RPM)
        BLdrive.set_velocity(SetVelocity_val, RPM)
        BRdrive.set_velocity(SetVelocity_val, RPM)
    elif str(perc_or_rpm) == "0":
        FLdrive.set_velocity(SetVelocity_val, PERCENT)
        FRdrive.set_velocity(SetVelocity_val, PERCENT)
        BLdrive.set_velocity(SetVelocity_val, PERCENT)
        BRdrive.set_velocity(SetVelocity_val, PERCENT)


def TurnDrivetrain(rotate):
    if str(rotate) == "LEFT":
        FLdrive.spin(REVERSE)
        FRdrive.spin(FORWARD)
        BLdrive.spin(REVERSE)
        BRdrive.spin(FORWARD)
    elif str(rotate) == "RIGHT":
        FLdrive.spin(FORWARD)
        FRdrive.spin(REVERSE)
        BLdrive.spin(FORWARD)
        BRdrive.spin(REVERSE)


def when_started():
    global Shoot, rpm_value, Rollervar, triggervar, autoroller, team_colour, Endgame, Orginal_Colour_Blue, vexcode_brain_precision, vexcode_console_precision, vexcode_controller_1_precision

    Colorchecker.set_light(LedStateType.ON)
    Colorchecker.set_light_power(50, PERCENT)
    controller_1.screen.print("Aut, ")
    brain.screen.set_cursor(1, 6)
    controller_1.screen.print("Off")
    XaxisRotation.set_position(49.5, DEGREES)
    Rollervar = 0
    Shoot = 0
    ShooterA.set_stopping(COAST)
    ShooterB.set_stopping(COAST)
    FLdrive.set_stopping(BRAKE)
    FRdrive.set_stopping(BRAKE)
    BLdrive.set_stopping(BRAKE)
    BRdrive.set_stopping(BRAKE)
    FLdrive.set_max_torque(60, PERCENT)
    FRdrive.set_max_torque(60, PERCENT)
    BLdrive.set_max_torque(60, PERCENT)
    BRdrive.set_max_torque(60, PERCENT)
    Rotationx.set_stopping(HOLD)
    Rotationx.set_max_torque(37, PERCENT)
    Rotationx.set_velocity(20, PERCENT)
    Roller.set_stopping(BRAKE)
    Roller.set_velocity(35, PERCENT)
    Roller.set_max_torque(100, PERCENT)
    ShooterA.set_max_torque(22, PERCENT)
    Colorchecker.gesture_disable()
    autoroller = 0
    team_colour = 0


def ondriver_drivercontrol_0():
    global Shoot, rpm_value, Rollervar, triggervar, autoroller, team_colour, Endgame, Orginal_Colour_Blue, vexcode_brain_precision, vexcode_console_precision, vexcode_controller_1_precision
    rpm_value = 450
    ShooterA.set_velocity(rpm_value, RPM)
    ShooterB.set_velocity(rpm_value, RPM)

    while True:
        # Drive
        BRdrive.set_velocity((controller_1.axis3.position() - (controller_1.axis1.position() - controller_1.axis4.position())), PERCENT)
        FRdrive.set_velocity((controller_1.axis3.position() - (controller_1.axis1.position() + controller_1.axis4.position())), PERCENT)
        BLdrive.set_velocity((controller_1.axis3.position() + (controller_1.axis1.position() - controller_1.axis4.position())), PERCENT)
        FLdrive.set_velocity((controller_1.axis3.position() + (controller_1.axis1.position() + controller_1.axis4.position())), PERCENT)
        FRdrive.spin(FORWARD)
        FLdrive.spin(FORWARD)
        BLdrive.spin(FORWARD)
        BRdrive.spin(FORWARD)
        # Spin up
        if controller_1.buttonX.pressing():
            Shoot = Shoot + 1
            wait(0.2, SECONDS)
        if Shoot % 2 == 1:
            ShooterA.spin(FORWARD)
            ShooterB.spin(FORWARD)
        else:
            ShooterA.stop()
            ShooterB.stop()
        # Roller auto
        if Colorchecker.color() == Color.RED and autoroller == 0:
            Roller.spin(FORWARD)
        elif Colorchecker.color() == Color.BLUE and autoroller == 0:
            Roller.stop()
        else:
            Roller.stop()
        if controller_1.buttonY.pressing():
            autoroller = autoroller + 1
            controller_1.screen.set_cursor(1, 1)
            controller_1.screen.print("Man, ")
            if autoroller == 2:
                controller_1.screen.set_cursor(1, 1)
                controller_1.screen.print("Aut, ")
                autoroller = 0
            wait(0.2, SECONDS)
        # Roller Toggle
        if autoroller == 1:
            if controller_1.buttonL2.pressing():
                Rollervar = Rollervar + 1
                wait(0.2, SECONDS)
            if Rollervar == 0:
                Roller.stop()
                controller_1.screen.set_cursor(1, 6)
                controller_1.screen.print("Off")
            elif Rollervar == 1:
                Roller.spin(FORWARD)
                controller_1.screen.set_cursor(1, 6)
                controller_1.screen.print("Fwd")
            else:
                if Rollervar == 2:
                    Rollervar = 0
        # Fire
        if controller_1.buttonR2.pressing():
            Trigger.set(True)
            # When should test to see the min val of wait
            wait(0.5, SECONDS)
            Trigger.set(False)
        # Rotation X axis
        if controller_1.buttonUp.pressing():
            Rotationx.spin(FORWARD)
        elif controller_1.buttonDown.pressing():
            Rotationx.spin(REVERSE)
        else:
            Rotationx.stop()
        # EndGame
        if controller_1.buttonL1.pressing():
            Blocker.set(False)
            wait(1, SECONDS)
        if controller_1.buttonLeft.pressing():
            EndgameB.set(True)
            wait(1, SECONDS)
            EndgameB.set(False)
        if controller_1.buttonRight.pressing():
            EndgameA.set(True)
            wait(1, SECONDS)
            EndgameA.set(False)
        wait(5, MSEC)
        # Odom
        if controller_1.buttonR1.pressing():
            odom.TurnToGoal()


def onauton_autonomous_0():
    global Shoot, rpm_value, Rollervar, triggervar, autoroller, team_colour, Endgame, Orginal_Colour_Blue, vexcode_brain_precision, vexcode_console_precision, vexcode_controller_1_precision
    odom.TurntoTargetHeading(-90)
    SetVelocity(50, "0")
    FLdrive.spin(REVERSE)
    FRdrive.spin(FORWARD)
    BLdrive.spin(FORWARD)
    BRdrive.spin(REVERSE)
    wait(5, MSEC)
    Roller.spin_for(FORWARD, 270, DEGREES, wait=True)
    Stop_Drivertrian()
    SetVelocity(50, "0")
    FLdrive.spin(FORWARD)
    FRdrive.spin(REVERSE)
    BLdrive.spin(REVERSE)
    BRdrive.spin(FORWARD)
    wait(50, MSEC)
    Stop_Drivertrian()
    odom.TurnToGoal()


class Odom():
    def __init__(self, starting_position):
        wait(1, SECONDS)
        self.starting_position = starting_position
        self.left_starting_position = (750, 415, 0)  # units in mm
        self.right_starting_position = (3436, 2980, 90)
        self.alliance_goal_location = (451, 3115)
        self.previous_RED = 0
        self.previous_LED = 0
        self.previous_BED = 0
        self.previous_theta = 0
        self.true_x = 0
        self.true_y = 0
        self.dtheta_dt = 0
        self.robot_heading_in_rad = 0
        self.robot_heading_in_deg = 0
        self.T = True
        self.Pre_Err = 0
        self.It = 0
        self.D = 0
        self.iterations = 0

    def updateOdomReadings(self):
        # local variables
        left_encoder_distance = 145
        right_encoder_distance = 145
        back_encoder_distance = 150

        odom_wheel_diameter = 219.6
        # drive_wheel_Diameter = 10 # this is not correct

        right_encoder_travel_distance = -(RightEncoder.position(DEGREES) / 360) * odom_wheel_diameter
        left_encoder_travel_distance = (LeftEncoder.position(DEGREES) / 360) * odom_wheel_diameter
        back_encoder_travel_distance = (BackEncoder.position(DEGREES) / 360) * odom_wheel_diameter

        # print("distances:", right_encoder_travel_distance, left_encoder_travel_distance, back_encoder_travel_distance)

        delta_RED = right_encoder_travel_distance - self.previous_RED
        delta_LED = left_encoder_travel_distance - self.previous_LED
        delta_BED = back_encoder_travel_distance - self.previous_BED

        # print("distances:", delta_LED, delta_RED)

        self.previous_RED = right_encoder_travel_distance
        self.previous_LED = left_encoder_travel_distance
        self.previous_BED = back_encoder_travel_distance

        theta = (delta_LED - delta_RED) / (left_encoder_distance + right_encoder_distance)

        # print(delta_LED - delta_RED)
        # print("theta: ", theta)

        delta_theta = theta - self.previous_theta
        self.previous_theta = theta

        if delta_theta == 0:
            x = delta_BED
            y = delta_RED
        else:
            x = 2 * (math.sin(delta_theta / 2) * ((delta_BED / delta_theta) + back_encoder_distance))
            y = 2 * (math.sin(delta_theta / 2) * ((delta_RED / delta_theta) + right_encoder_distance))

        ThetaTransform = self.previous_theta + (delta_theta / 2)

        # converting to polar coordinates
        gamma = math.atan2(y, x)
        radius = math.sqrt(x ** 2 + y ** 2)

        new_gamma = gamma + (-ThetaTransform)

        # converting back to cartesian coords after global transformation
        deltax = radius * math.cos(new_gamma)
        deltay = radius * math.sin(new_gamma)

        # satys withing +-pi (-3pi/2 --> pi/2)
        theta += math.pi
        while theta <= 0:
            theta += 2 * math.pi
            theta = theta % (2 * math.pi)
        theta -= math.pi

        self.previous_theta = theta
        self.robot_heading_in_rad += theta
        self.robot_heading_in_deg = (self.robot_heading_in_rad * (180 / math.pi))
        # print(self.robot_heading_in_deg)

        self.true_x = self.true_x + deltax
        self.true_y = self.true_y - deltay

        # print("\n\nCOORDS:, x =", round(self.true_x), ", y =",round(self.true_y),"robot_heading_in_deg:", round(self.robot_heading_in_deg), "\n\n")

        wait(10, MSEC)

    def getOdomValues(self):
        self.updateOdomReadings()
        if self.starting_position == "left":

            return (self.true_x + self.left_starting_position[0]), (self.true_y + self.left_starting_position[1]), (self.robot_heading_in_deg + self.left_starting_position[2])
        elif self.starting_position == "right":
            return (self.true_x + self.left_starting_position[0]), (self.true_y + self.right_starting_position[1]), (self.robot_heading_in_deg + self.right_starting_position[2])

    def PIDController(self, TargetCoords, CurrentCoords, target_heading, robot_heading_in_deg_in):
        # PID constants
        Kp = 0.6
        Ki = 0.3
        Kd = 0.1

        if target_heading is None:
            pass
        elif TargetCoords is None:
            Err = target_heading - robot_heading_in_deg_in
            # print("\nErr/Target/PV:",Err, target_heading, robot_heading_in_deg_in)

            # I don't know why i didn't set to zero but yea it worked !! :)
            if self.Pre_Err == 0:
                self.Pre_Err = Err

            # region The PID calculation:
            P = Kp * Err
            self.It = self.It + (Err * Ki * 0.01)
            # the dirivative is only changed every 10 * 10ms
            if self.iterations % 10 == 0:
                # Averaging
                self.D = Kd * ((Err - self.Pre_Err) / 1)

            Out = round(P + self.D + self.It, 2)
            # print("Out/P/I/D:", Out, P, self.It, self.D, "\n")

            # endregion End of the PID Calc

            # Error basically never reaches 0.00000 so the value is rounded
            if round(Err) != 0:
                SetVelocity(1.5 * Out, "rpm")
                TurnDrivetrain("RIGHT")

                # ignore this (it works when i tested it :), trust me)
                Err = self.Pre_Err

                self.iterations += 1
                return True
            else:
                Stop_Drivertrian()
                return False

    def getTargetHeadingToGoal(self):
        # This calculates which way the robot should turn relative to the pitch and sets a target heading
        x, y, robot_heading_in_deg_got = self.getOdomValues()
        delta_x = abs(self.alliance_goal_location[0]) - abs(x)
        delta_y = abs(self.alliance_goal_location[1]) - abs(y)
        angle_from_horizontal_to_robot = (math.atan2(delta_y, delta_x)) * (180 / math.pi)

        if delta_x > 0 and delta_y > 0:
            target_heading = 90 - angle_from_horizontal_to_robot
        elif delta_x < 0 and delta_y > 0:
            target_heading = -90 + angle_from_horizontal_to_robot
        elif delta_x < 0 and delta_y < 0:
            target_heading = -180 - angle_from_horizontal_to_robot
        elif delta_x == 0:
            target_heading = 180

        # print(angle_from_horizontal_to_robot)
        # print(target_heading)

        return -target_heading

    def TurntoTargetHeading(self, target_heading):
        self.It = 0
        self.T = True
        while self.T:
            # gets controller joystick position
            controller_joystick_1_pos = controller_1.axis1.position()
            controller_joystick_3_pos = controller_1.axis3.position()
            # print(round(controller_joystick_1_pos), round(controller_joystick_3_pos))

            # After testing I can't get the "-5 < controller_joystick_1_pos < 5" to work so I split it into two if statments
            # It essentially creates an override to the odometry from user input
            if (-5 < controller_joystick_1_pos and controller_joystick_1_pos < 5):
                if (-5 < controller_joystick_3_pos and controller_joystick_3_pos < 5):
                    _, _, robot_heading_in_deg_got = self.getOdomValues()

                    self.T = self.PIDController(None, None, target_heading, robot_heading_in_deg_got)
                    print("Target vs Real:", target_heading, robot_heading_in_deg_got, "\n")
                    wait(10, MSEC)
                else:
                    break
            else:
                break
        print("\n**Ended**\n")

    def TurnToGoal(self):
        target_heading = self.getTargetHeadingToGoal()
        self.TurntoTargetHeading(target_heading)


# create a function for handling the starting and stopping of all autonomous tasks
def vexcode_auton_function():
    # Start the autonomous control tasks
    auton_task_0 = Thread(onauton_autonomous_0)
    # wait for the driver control period to end
    while (competition.is_autonomous() and competition.is_enabled()):
        # wait 10 milliseconds before checking again
        wait(10, MSEC)
    # Stop the autonomous control tasks
    auton_task_0.stop()


def vexcode_driver_function():
    # Start the driver control tasks
    driver_control_task_0 = Thread(ondriver_drivercontrol_0)

    # wait for the driver control period to end
    while (competition.is_driver_control() and competition.is_enabled()):
        # wait 10 milliseconds before checking again
        wait(10, MSEC)
    # Stop the driver control tasks
    driver_control_task_0.stop()


# instantiate classes
odom = Odom(starting_position="left")

# register the competition functions
competition = Competition(vexcode_driver_function, vexcode_auton_function)

when_started()

# keeps the Odom updating
while competition.is_enabled():
    wait(10, MSEC)
    odom.updateOdomReadings()
