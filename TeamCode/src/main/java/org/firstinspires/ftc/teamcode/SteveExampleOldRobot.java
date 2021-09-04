/*
Copyright 2019 FIRST Tech Challenge Team 17235

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

PURPOSE:
This code is example code, meant to show a few different concepts.  The code configures the
expansion hub resources in a way that matches the Mr COD robot from 2019.
FUNCTION:
1. Sets up a TeleOp mode that allows for the control of the robot's motion
2. Sets up a state machine that supports basic motion as well as example code for creating driver
      assist functions
3. Shows some of the Java basics, like creating specific classes for different subsystems
*/
package org.firstinspires.ftc.teamcode;

/*
This section defines all of the Java packages required for using the different FTC resources
NOTE: This is something you will need for every OpMode you create.  It is also something that
can be put together into a separate package/file that you can just include in every block of
code you write.
 */
import com.qualcomm.hardware.bosch.BNO055IMU;  //This is the package for controlling the IMU
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

  //This is the standard Java package for a variety of math functions

/**
 * This file contains a minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode allows you to have basic control of your robot during Driver Mode.
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */
@TeleOp

public class SteveExampleOldRobot extends LinearOpMode {
    /*
    This section maps the resources that you defined in the FTC Controller configuration.
    These are variable declarations, where you are assigning the resource to the corresponding
    Java package for the resource.
    NOTE: This is something you will need for every OpMode you create.  It is also something that
    can be put together into a separate package/file that you can just include in every block of
    code you write.  This section should probably be standardized for your all of your projects
    since you all are using the same robot.
    Note:  The name of this class will be what shows up on your controllers.
     */
    //Declaration of the two expansion hubs
    private Blinker expansion_Hub_1;
    private Blinker expansion_Hub_2;
    //Declaration for the servo for the puller
    private Servo back_foundation_puller;
    //Declaration for the four drive motors
    private DcMotor back_left_wheel;
    private DcMotor back_right_wheel;
    private DcMotor front_left_wheel;
    private DcMotor front_right_wheel;
    //Declaration for the sensors.  These classes were created to work with the default/supported
    //sensors on the robot.  Technically, if we ever run into a sensor that has the same interface
    //as one of the ports on the Expansion Hub, you could design your own interface.
    private DistanceSensor color_sensor;
    private DistanceSensor distance_sensor;
    private ColorSensor front_color_sensor;
    //Declaration for the Inertial Measurement Unit (IMU)--note:  This is the built in gyroscope
    //in the Expansion Hub.
    private BNO055IMU imu;
    private DcMotor lift_motor;
    private DcMotor clamp_motor;
    //Below is the declaration for the digital "red/blue" switch we have on our old robot.
    //The system has a collection of other interfaces that can be used regularly.
    //NOTE:  For those of you using motion control/needing start/stop locations, you could use
    //something like this for a limit switch or something--and then use it as part of a closed loop
    //control algorithm.
    private DigitalChannel switch_;

    /*
    CLASSES
    Note:  The next section declares a couple of classes.  For more information on the parts of a
    class, please refer to https://www.developer.com/java/ent/article.php/3488176/The-Components-of-a-Class.htm
    Note2: You will probably see that I've made some mistakes with my use of "public" and "private".
    You can get a really good description of correct practice at:
    https://therenegadecoder.com/code/the-difference-between-private-and-public-in-java/
    If you find mistakes in what I've done, CHANGE THEM!  You can also take a pass at calling them
    out publicly--nothing like a little shame to ensure the coach is doing things correctly.
    (If you do make changes, though, test them--nothing screws up your flexing like checking in bad
    code!)
     */

    /*
    Class: Chassis
    Attributes:
        frontLeft: variable representing the desired/current speed of the front left motor
        backLeft:  variable representing the desired/current speed of the back left motor
        frontRight: variable representing the desired/current speed of the front right motor
        backRight: variable representing the desired/current speed of the back right motor
     Accessors:
        SetMotors: This function takes arguments drive, strafe, rotate; translates them into
            values the motors will recognize; and saves them in the corresponding attributes.
        Drive:  This function takes the values of the attributes and sets the motor power.
     */
    private class Chassis {
        double frontLeft;
        double backLeft;
        double frontRight;
        double backRight;
        
        private void SetMotors (double drive, double strafe, double rotate) {
            this.frontLeft = -drive + strafe + rotate;
            this.backLeft = -drive - strafe + rotate;
            this.frontRight = drive + strafe + rotate;
            this.backRight = drive - strafe + rotate;
        }

        private void Drive () {
            front_left_wheel.setPower(this.frontLeft);
            back_left_wheel.setPower(this.backLeft);
            front_right_wheel.setPower(this.frontRight);
            back_right_wheel.setPower(this.backRight);
        }
    }

    /*
    Class: UberTool
    Attributes: None
    Accessors:
        Lift: This sets the power of the scissors lift motor to the value of the argument liftPower
        Clamp: This sets the power of the clamp motor to the value of the argument clampPower
     */
    private class UberTool {
        private void Lift (double liftPower) {
            lift_motor.setPower(liftPower);
        }

        private void Clamp (double clampPower) {
            clamp_motor.setPower(clampPower);
        }
    }

    /*
    OperState is a enumerated type that is used to define the states in the main OpMode.
    Below, the variable driveOpState will be declared to be of this time and the states will be
    interpreted as follows:
    NORMALDRIVE:  When in this state, the software will give the controller full ability to move
        everything from the chassis motors to the lift/clamp.  This is where the "default" behavior
        of the system is supported.  Based on certain controller actions, the system can be moved
        to other states.
    ROTATEPOSITION: When in this state, the software will "automatically" control the robot,
        rotating it until the robot is facing a specific angle relative to the starting position.

     Due to the way Java handles enum classes, I was not able to get this type to be recognized
     anywhere else in the code--I'm still learning why, but it appears to be due to some gaps in
     the way Java supports inheritance of enums.
     Activity for the reader:  teach the team why this is!
     */
    enum OperState {
        NORMALDRIVE,
        ROTATETOPOSITION
    }

    /*
    This is the main accessor (function) for the whole OpMode.
    You will write most of your custom code here.
     */
    @Override
    public void runOpMode() {
        /*
        INITIALIZATION SECTION
        The initialization section in this code doesn't do much--just configs the robot resources
        and initializes the IMU.
         */
        /*
        ROBOT RESOURCE ACCESS

        For now, let's call these "magic functions".  I'm not sure exactly what's happening in these,
        but they are required for accessing the FTC resources.
        The names in the deviceName argument must EXACTLY match those in the configuration on your
        controller.
         */
        expansion_Hub_1 = hardwareMap.get(Blinker.class, "Expansion Hub 1");
        expansion_Hub_2 = hardwareMap.get(Blinker.class, "Expansion Hub 2");
        back_foundation_puller = hardwareMap.get(Servo.class, "back foundation puller");
        back_left_wheel = hardwareMap.get(DcMotor.class, "back left wheel");
        back_right_wheel = hardwareMap.get(DcMotor.class, "back right wheel");
        clamp_motor = hardwareMap.get(DcMotor.class, "clamp motor");
        color_sensor = hardwareMap.get(DistanceSensor.class, "color sensor");
        distance_sensor = hardwareMap.get(DistanceSensor.class, "distance sensor");
        front_color_sensor = hardwareMap.get(ColorSensor.class, "front color sensor");
        front_left_wheel = hardwareMap.get(DcMotor.class, "front left wheel");
        front_right_wheel = hardwareMap.get(DcMotor.class, "front right wheel");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        lift_motor = hardwareMap.get(DcMotor.class, "lift motor");
        switch_ = hardwareMap.get(DigitalChannel.class, "switch ");

        /*
        IMU CONFIGURATION
         */
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();  //in wrong spot--where is better?
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu.initialize(parameters);

        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        /*
        END OF INITIALIZATION SECTION
         */
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        /*
        MAIN SECTION FOR MATCH
         */
        // run until the end of the match (driver presses STOP)
        /*
        Variable declarations that are used by software.
        Note:  I left out public/private/static declarations--what would be appropriate here?
         */
        double liftPower;  //passed as an argument for lift motor power
        double clampPower; //passed as an argument for clamp motor power
        double drive;  //Power for forward and back motion
        double strafe; //Power for left and right motion
        double rotate; //Power for rotating the robot
        Chassis superRobot = new Chassis(); //instantiation of chassis class
        UberTool bigTool = new UberTool(); //instantiation of tool class
        double rotationAngle = 0;  //passed as state argument for the set position in driveOpState
        OperState driveOpState = OperState.NORMALDRIVE; //instantiation of the state variable
        /*
        Note: In this example, I don't do any other "first time" operations on the software--the
        code immediately drops into the while loop.  If you wanted to do some additional post-PLAY
        initialization, you would do it here before going to the while loop.
         */

        /*
        MAIN WHILE LOOP
        This is the main loop where all of the work is done.
        ONE KEY CONSIDERATION:
        This loop will be traversed continuously--it represents a slice of time of the robot's overall
        performance.  Keep in mind that we want this loop to take as close to 0 time as possible.
        Running most code will take nearly 0 time.  Each sensor you read will take between 3-7 ms.
        Again, that's not much unless you start reading everything every time you go through a loop.
        The faster this loop is, the more responsive your code will be.
        If you start seeing slow response from your robot when you're using it, it's probably
        because it takes too much time to process the code in this loop.
        ***This concern is a major reason for using a state machine in the loop--that way, you're
        only ever looking at the stuff you really care about.
         */
        while (opModeIsActive()) {
            /*
            UNIVERSAL ACTIONS
            These actions will happen every time you go through the while loop.
            As noted above, keep in mind that we'll want to keep these to a minimum.
             */
            telemetry.addData("Status", "Let's get GOING!!!");
            /*
            IMU angle reading
            There are more things that can be done than just reading the angles--and I only use the
            Z angle for this example.
            The IMU is actually pretty powerful--you can also measure acceleration and some other
            things.  As an example, you might be able to use the IMU to detect when you hit a wall
            at full speed.
             */
            double zAngle = imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES).firstAngle;
            double yAngle = imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES).secondAngle;
            double xAngle = imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES).thirdAngle;
            /*
            MAIN STATE MACHINE
            This is the main state machine for the system.
            The power of a state machine is that you can get the robot to do different things
            in each pass through the while loop based on the situation (or state) within the system.
            Each state is made up of two main sections:  OPERATIONS and STATE MANAGEMENT
             */
            switch (driveOpState) {
                case NORMALDRIVE :
                    /*
                    STATE:  NORMALDRIVE
                    OPERATIONS:
                    This is the default state for the system (the state variable is pre-set to this
                    when it's instantiated above).
                    Each time the code in this state is run, it will read the current values from
                    the gamepad and take action accordingly.
                    Note that this state will always set the chassis motor power and always set the
                    lift/clamp motor power.
                     */
                    drive = -this.gamepad1.left_stick_y;
                    strafe = -this.gamepad1.left_stick_x;
                    rotate = -this.gamepad1.right_stick_x;
                    liftPower = -this.gamepad1.right_stick_y;
                    clampPower = 0.25*(-this.gamepad1.right_trigger + this.gamepad1.left_trigger);
                    superRobot.SetMotors (drive, strafe, rotate);
                    superRobot.Drive();
                    bigTool.Lift(liftPower);
                    bigTool.Clamp(clampPower);
                    /*
                    STATE MANAGEMENT
                    If the left or right bumper are detected, then the desired rotation angle is
                    set and the state machine is changed to ROTATEPOSITION.  Otherwise, stay in
                    the NORMALDRIVE state by default.

                    Note:  This is hierarchical.  If I press both left and right bumpers at the same
                    time, then the left_bumper "wins."  The net effect:  this determines the
                    priority of our system.
                    This has tradeoffs, but one big advantage is that you could use the "top"
                    conditional to create an "abort" for safety.
                     */
                    if (this.gamepad1.left_bumper) {
                        driveOpState = OperState.ROTATETOPOSITION;
                        rotationAngle = -45;
                    } else if (this.gamepad1.right_bumper) {
                        driveOpState = OperState.ROTATETOPOSITION;
                        rotationAngle = 60;
                    } else {
                        driveOpState = OperState.NORMALDRIVE;
                    }
                    break;
                case ROTATETOPOSITION :
                    /*
                    STATE: ROTATETOPOSITION
                    OPERATIONS and STATE MANAGEMENT:
                    If the measured angle of rotation is within 2 degrees of desired
                    rotationAngle, then stop the motors and change the state to NORMALDRIVE.
                    Otherwise, set the rotational speed and direction based on difference between
                    the measured angle and the desired rotationAngle.  This has the net effect on
                    the system of having the robot rotate quicker when it's far away from its
                    objective and slower (more precisely) when it's close to the objective.
                    Note:  a minimum power of 0.15 is guaranteed because the motors don't move the
                    robot at a speed less than that.
                    Note:  This is an example of a really simple closed loop control system.
                    "Closed loop" means that we are determining the next thing we want to do by
                    looking at what the system is currently doing--in this case, I'm picking my
                    motor speed based on how far away the robot is from its destination.
                    Control systems is a whole field of expertise that give you advanced algorithms
                    for doing "closed loop" control.  The most famous of these methods is the PID
                    controller.  This is an advanced method, with simpler varients like PI controllers,
                    that can help you do things more quickly than a simple algorithm like the one I
                    used here.
                     */
                    if (Math.abs(zAngle - rotationAngle) < 2) {
                        superRobot.SetMotors(0,0,0);
                        superRobot.Drive();
                        driveOpState = OperState.NORMALDRIVE;
                    } else {
                        rotate = Math.max(0.15, Math.abs(rotationAngle - zAngle) / 180);
                        rotate = Math.signum(rotationAngle - zAngle) * rotate;
                        superRobot.SetMotors(0, 0, rotate);
                        superRobot.Drive();
                        driveOpState = OperState.ROTATETOPOSITION;
                    }
                    break;
                default :
                    break;

            }

            /*
            MORE UNIVERSAL ACTIONS
            In the while loop, out of the state machine, this is another area where you can add
            stuff that will be run every time the code runs through the while loop.
            Note:  This is the only place where I call the telemetry.update function--that's a
            pretty expensive function with respect to time.
             */
            telemetry.addData("MyRotation", zAngle);
            telemetry.addData("Current State: ", driveOpState);
            telemetry.update();
            

        }
    }
}
