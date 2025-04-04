package org.firstinspires.ftc.team13590;

// hardware & class imports
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.*;

public class RobotHardware {

    // Declare OpMode members
    protected final LinearOpMode myOpMode; // gain access to methods in the calling OpMode.
    public final double MtoIN = 39.3701; // meters to inches #

    // Define Motor and Servo objects (Make them private so that they CANT be accessed externally)
    public DcMotor leftFrontDrive = null;
    public DcMotor leftBackDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor rightBackDrive = null;
    public DcMotor elbowDrive = null;
    public DcMotor extensionDrive = null;
    public Servo clawPinch = null;
    public Servo clawYaw = null;
    public Servo clawAxial = null;
    public Limelight3A limelight = null;

    // Define Sensor objects (Make them private so that they CANT be accessed externally)
    public IMU imu = null; // Universal IMU interface

    public int coloredSoundID;
    public int yellowSoundID;
    public boolean yellowFound;
    public boolean coloredFound;

    /*
    These variables are declared here (as class members) so they can be updated in various methods, but still be
    displayed by sendTelemetry()
     */
    public double heading; // yaw of robot

    // Rudimentary initialization of variables
    public double drivePower;
    public double strafePower;
    public double turnPower;

    public double leftFrontPower;
    public double leftBackPower;
    public double rightFrontPower;
    public double rightBackPower;


    public double DRIVE_SPEED;
    public double STRAFE_SPEED;
    public double TURN_SPEED;
    public double ELBOW_SPEED;

    public double CLAW_CLOSE;
    public double CLAW_OPEN;
    public double CLAW_COLLAPSED;
    public double CLAW_DOWN;
    public double CLAW_MID;
    public double CLAW_UP;
    public double CLAW_RAISED;
    public double CLAW_IN;
    public double CLAW_OUT;
    public double YAW_MID;
    public double YAW_LEFT;
    public double YAW_RIGHT;
    // End of Rudimentary inits...


    // initialize enums ; easier to read :D
    public enum statesOfBeing {
        ACTIVATE,
        DEACTIVATE,
        PASS,
        SUPERPOSITION
    }

    // initialize enum constants ; this is for passing enum values into other classes
    public statesOfBeing enable = statesOfBeing.ACTIVATE;
    public statesOfBeing disable = statesOfBeing.DEACTIVATE;
    public statesOfBeing superposition = statesOfBeing.SUPERPOSITION;
    public statesOfBeing pass = statesOfBeing.PASS;

    // Declare Wheel Encoder Variables
    public int leftFrontTarget;
    public int leftBackTarget;
    public int rightFrontTarget;
    public int rightBackTarget;


    public final byte ROBOT_WIDTH = 17;
    public final byte ROBOT_LENGTH = 13;
    public final double ROBOT_DIAG_RADIUS = Math.sqrt(Math.pow(ROBOT_LENGTH/2f, 2) + Math.pow(ROBOT_WIDTH/2f, 2));

    public double COUNTS_PER_MOTOR_REV = 537.7;
    public double WHEEL_DIAMETER_INCHES = 3.77953;
    public double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * Math.PI);

    // Declare Extender Encoder Variables
    public final double EXTENSION_COUNTS_PER_REV =
        28 // counts for bare motor revolution (aka int number at the end of the *encoder resolution formula*
            * (   (1+(46./17)) ) // times internal gearing (aka *gear ratio formula*)
            * (60. / 100.) // external gearing, 100 (drive) to 60 teeth
            * (1.0); // ... per revolution ( simplified from 360/360 like the logic from the Elbow Count formula)

    public final double EXTENSION_INCH_PER_REV = 0.375;
    // by the distance traveled

    public final double EXTENSION_REVS_PER_INCH = 3;
    public final double EXTENSION_COUNTS_PER_INCH = EXTENSION_REVS_PER_INCH*EXTENSION_COUNTS_PER_REV; // Find the inches per rev, then multiply EXTENSION_COUNTS_PER_REV
    public final double EXTENSION_MAXIMUM_COUNT = (EXTENSION_COUNTS_PER_REV * (27.3)); // the other number is how many revs
            // it takes for the linear actuator to reach the top. the -(#) is the amount of revs for tolerance

    public final double FORWARD_EXTENSION_LIMIT = (EXTENSION_MAXIMUM_COUNT); // FIXME Placeholder; farthest extender can go forwards
    public final double REARWARD_EXTENSION_LIMIT = 2.3*EXTENSION_COUNTS_PER_INCH; // FIXME measured or calculated; farthest the extender can go backwards before going OOB

    // Declare Elbow Encoder Variables, REMEMBER TO DECLARE WHEEL ONES LATER!!
    public final double ARM_COUNTS_PER_DEGREE =
        28 // counts for motor revolution...
            * (250047.0 / 4913.0) // times internal gearing (yes, counts per motor rev are the BARE drive)
            * (100.0 / 20.0) // external gearing, 20 to 100 teeth
            * (1.0 / 360.0); // ... per degree
    public final int ELBOW_ANGLE_OFFSET = 37; //  Should be the angle from parallel to floor to the true zero
    public final int ELBOW_TRUE_OFFSET = 127; //  Should be from true zero to perpendicular

    /* Elbow Positions
     ELBOW_ANGLE_OFFSET is the offset angle the arm starts off on
     the numbers added after it are tweaks for more accurate positions due to gravity and such
     Rounding converts the value into a long (dk why) so it shows up like 90.0, add (int) to convert

     VARIABLES CONCERNING ANGLES HAVE THIS RULE APPLIED:

     TRUE angles are angles RELATIVE to the MOTOR STARTING POINT, NOT TO YOUR PREFERRED PLANE OF REFERENCE
     STANDARD angles, or angles w/o the TRUE suffix are angles BASED on the PLANE OF REFERENCE

     SN: these variables declared here are the only ones that are exempt to this rule as they are used when stating
         positions in the executive code and I do not wish to cause confusion there.
     */
    public double ELBOW_COLLAPSED = 0;
    public double ELBOW_PARALLEL = Math.round((ELBOW_ANGLE_OFFSET) * ARM_COUNTS_PER_DEGREE);
    public double ELBOW_ANGLED = Math.round((45 + ELBOW_ANGLE_OFFSET) * ARM_COUNTS_PER_DEGREE);
    public double ELBOW_PERPENDICULAR = Math.round((90 + ELBOW_ANGLE_OFFSET) * ARM_COUNTS_PER_DEGREE);
    public double ELBOW_BACKWARD_PARALLEL = Math.round((180 + ELBOW_ANGLE_OFFSET) * ARM_COUNTS_PER_DEGREE);
    public double ELBOW_FUDGE_FACTOR = 5 * ARM_COUNTS_PER_DEGREE; // Amount to rotate the elbow by
    public double angleConvert(double angle){
        return Math.round((angle) * ARM_COUNTS_PER_DEGREE);
    }

    /**
     * Returns count position for elbow when moving extender to keep it on the same y level.
     * @return elbow position (in counts, needs casting) relative to extender position to keep the extender leveled
     */
    public double armByExtender() { // returns elbow pos for extension pos; this sould be a positive slope: as extender goes up arm goes up
        // 13x/25 + 22 = y  ; x = extension pos, y = elbow pos
        return (((((extensionDrive.getCurrentPosition()/EXTENSION_COUNTS_PER_REV)*13)/25) + 5) * ARM_COUNTS_PER_DEGREE);// slope goes here: initial position of 0, 22 ; 25, 35
    }

    public double extenderByArm() { // this should be a negative slope: as arm goes up extender goes down
        return (elbowDrive.getCurrentPosition());// slope goes here: initial position of arm (parallel), extend (0) ; final position (up or down)
    }

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public RobotHardware(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Initialize all the robots' hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All the hardware devices are accessed via the hardware map and initialized.
     */
    public void init(boolean auto) {

        // Define and initialize ALL installed motors (note: need to use reference to the actual OpMode).
        leftFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "left_front_drive"); // Stuff on CH
        leftBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_back_drive");

        elbowDrive = myOpMode.hardwareMap.get(DcMotor.class, "elbow_drive"); // Stuff on EH
        extensionDrive = myOpMode.hardwareMap.get(DcMotor.class, "extension_drive");
        clawPinch = myOpMode.hardwareMap.get(Servo.class, "claw_pinch");
        clawYaw = myOpMode.hardwareMap.get(Servo.class, "claw_yaw");
        clawAxial = myOpMode.hardwareMap.get(Servo.class, "claw_axial");

        limelight = myOpMode.hardwareMap.get(Limelight3A.class, "limelight-rfc");

        DRIVE_SPEED = 0.5; // Maximum autonomous driving speed for better distance accuracy.
        STRAFE_SPEED = 0.5; // Maximum autonomous strafing speed for better distance accuracy.
        TURN_SPEED = 0.4; // Maximum autonomous turning speed for better rotational accuracy.
        ELBOW_SPEED = 1.0; // Maximum elbow speed.

        CLAW_CLOSE = 0.4; // TBD
        CLAW_OPEN = 0.1; // TBD

        CLAW_UP = 0.92; // MAXIMUM, not playable position
        CLAW_MID = 0.51; // playable position
        CLAW_DOWN = 0.29; // playable position
        CLAW_COLLAPSED = 0.08; // MAXIMUM, not playable position

        YAW_LEFT = 0.0;
        YAW_MID = 0.37;
        YAW_RIGHT = 1.0;

        /*
            Playable positions, initialize them if you feel the need to do so in the future:
            CLAW_BOTTOM = 0.3; // Claw at collapsed position
            CLAW_ANGLED = 0.67; // Claw at angled position
            CLAW_ANTI_BOTTOM = 0.72 // Claw at anti collapsed position
            Not initialized because you can just call the calibrateClaw(); function
         */

        CLAW_IN = 0.0; // TBD
        CLAW_OUT = 0.6; // TBD

        // Define and initialize ALL installed sensors (note: need to use reference to the actual OpMode).
        imu = myOpMode.hardwareMap.get(IMU.class, "imu");
        // imu = myOpMode.hardwareMap.get(IMU.class, "imu2");

        /*
         Define how the hub is mounted on the robot to get the correct Yaw, Pitch, and Roll values. There are two input
         parameters required to fully specify the orientation. (1) the first parameter specifies the direction of the
         printed logo on the hub is pointing. (2) the second parameter specifies the direction the USB connector on the
         hub is pointing. All directions are relative to the robot, and left/right is as-viewed from behind the robot.
         */



        /*
         Most robots need the motors on one side to be reversed to drive forward. The motor reversals shown here are
         for a "direct drive" robot (the wheels turn in the same direction as the motor shaft). If your robot has
         additional gear reductions or uses a right-angled drive, it is important to ensure that your motors are turning
         in the correct direction. So, start out with the reversals here, BUT when you first test your robot, push the
         left joystick forward and observe the wheels turn. Reverse the direction (flip FORWARD <-> REVERSE) of any
         wheel that runs backward. Keep testing until ALL the wheels move the robot forward when you push the left
         joystick forward.
         */
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        elbowDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        extensionDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        elbowDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extensionDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ensure elbow starts at 0
        elbowDrive.setTargetPosition(0);
        elbowDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extensionDrive.setTargetPosition(0);
        extensionDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (auto){
            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                    RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));

            imu.initialize(parameters);

            // Reset the IMU when initializing the hardware class
            imu.resetYaw();

            clawAxial.setPosition(CLAW_DOWN);
            clawPinch.setPosition(CLAW_CLOSE);
            clawYaw.setPosition(YAW_MID);
            elbowDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            extensionDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        elbowDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extensionDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        heading = imu.getRobotYawPitchRollAngles().getYaw();


        // initialize limelight
        limelight.start();

        // Initialize sound player
        coloredSoundID = myOpMode.hardwareMap.appContext.getResources().getIdentifier("colored", "raw", myOpMode.hardwareMap.appContext.getPackageName());
        yellowSoundID = myOpMode.hardwareMap.appContext.getResources().getIdentifier("yellow", "raw", myOpMode.hardwareMap.appContext.getPackageName());
        // preload sounds
        if (coloredSoundID != 0) {
            coloredFound = SoundPlayer.getInstance().preload(myOpMode.hardwareMap.appContext, coloredSoundID);
        }
        if (yellowSoundID != 0) {
            yellowFound = SoundPlayer.getInstance().preload(myOpMode.hardwareMap.appContext, yellowSoundID);
        }
        myOpMode.telemetry.addData("gold resource", coloredFound ? "Found" : "NOT found\n Add colored.wav to /src/main/res/raw");
        myOpMode.telemetry.addData("silver resource", yellowFound ? "Found" : "Not found\n Add yellow.wav to /src/main/res/raw");
        SoundPlayer.getInstance().setMasterVolume(4);

        // Wait for the game to start (Display Gyro value while waiting)



        while (myOpMode.opModeInInit()) {
            myOpMode.telemetry.addData("Status", "Hardware Initialized");
            myOpMode.telemetry.addData("Wheels starting at", "%7d :%7d :%7d :%7d",
                    leftFrontDrive.getCurrentPosition(), leftBackDrive.getCurrentPosition(),
                    rightFrontDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
            myOpMode.telemetry.addData("Starting Elbow Pos:", elbowDrive.getCurrentPosition());
            myOpMode.telemetry.update();
        }
    }

    /**
     * Calculate the motor powers required to achieve the requested robot motions:
     * Drive (Axial motion), Strafe (Lateral motion), and Turn (Yaw motion)
     * Then send these power levels to the motors.
     *
     * @param drive     Forward/Reverse driving power (-1.0 to 1.0) +ve is forward
     * @param strafe    Right/Left driving power (-1.0 to 1.0) +ve is right
     * @param turn      Right/Left turning power (-1.0 to 1.0) +ve is clockwise
     */
    public void driveFieldCentric(double drive, double strafe, double turn) {

        drivePower = drive;
        strafePower = strafe;
        turnPower = turn;

        double max;
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot rotation
        double strafeRotation = strafe * Math.cos(-botHeading) - drive * Math.sin(-botHeading);
        double driveRotation = strafe * Math.sin(-botHeading) + drive * Math.cos(-botHeading);

        /*
         Combine the joystick requests for each axis-motion to determine each wheel's power. Set up a variable for each
         drive wheel to save the power level for telemetry. Denominator is the largest motor power (absolute value) or
         1. This ensures all the powers maintain the same ratio, but only when at least one is out of the range [-1, 1]
         */
        double denominator = Math.max(Math.abs(driveRotation) + Math.abs(strafeRotation) + Math.abs(turn), 1);
        leftFrontPower = (driveRotation + strafeRotation + turn) / denominator;
        leftBackPower = (driveRotation - strafeRotation + turn) / denominator;
        rightFrontPower = (driveRotation - strafeRotation - turn) / denominator;
        rightBackPower = (driveRotation +  strafeRotation - turn) / denominator;

        // Normalize the values so no wheel power exceeds 100%.
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            leftBackPower /= max;
            rightFrontPower /= max;
            rightBackPower /= max;
        }

        // Use existing function to drive all wheels.
        setDrivePower(leftFrontPower, leftBackPower, rightFrontPower, rightBackPower);
    }

    public void driveRobotCentric(double drive, double strafe, double turn){
        double max;

        // Combine drive and Turn for blended motion.
        double leftFPower = drive + strafe + turn;
        double leftBPower = drive - strafe + turn;
        double rightFPower = drive - strafe -turn;
        double rightBPower = drive + strafe -turn;

        // Scale the values so neither exceed +/- 1.0
        max = Math.max(Math.abs(leftFPower) , Math.abs(leftBPower));
        max = Math.max(max, Math.abs(rightFPower));
        max= Math.max(max, Math.abs(rightBPower));

        if (max > 1.0){
            leftFPower /= max;
            leftBPower /= max;
            rightFPower /= max;
            rightBPower /= max;
        }

        //Use existing function to drive both wheels.
        setDrivePower (leftFPower, leftBPower, rightFPower, rightBPower);
    }

    /**
     * Pass the requested wheel motor power to the appropriate hardware drive motors.
     *
     * @param leftFrontWheel    Forward/Reverse driving power (-1.0 to 1.0) +ve is forward
     * @param leftBackWheel     Forward/Reverse driving power (-1.0 to 1.0) +ve is forward
     * @param rightFrontWheel   Forward/Reverse driving power (-1.0 to 1.0) +ve is forward
     * @param rightBackWheel    Forward/Reverse driving power (-1.0 to 1.0) +ve is forward
     */
    public void setDrivePower(double leftFrontWheel, double leftBackWheel,
                              double rightFrontWheel, double rightBackWheel) {
        //Output the values to the motor drives.
        leftFrontDrive.setPower(leftFrontWheel);
        leftBackDrive.setPower(leftBackWheel);
        rightFrontDrive.setPower(rightFrontWheel);
        rightBackDrive.setPower(rightBackWheel);
    }

    public void encoderFieldCentric(double driveIN, double strafeIN, double turnDEG){
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot rotation
        double strafeRotation = (strafeIN*COUNTS_PER_INCH) * Math.cos(-botHeading) - (driveIN*COUNTS_PER_INCH) * Math.sin(-botHeading);
        double driveRotation = (strafeIN*COUNTS_PER_INCH) * Math.sin(-botHeading) + (driveIN*COUNTS_PER_INCH) * Math.cos(-botHeading);

        // sector length formula (rad*radius)
        double turnTICKS = (Math.toRadians(turnDEG) * ROBOT_DIAG_RADIUS) *COUNTS_PER_INCH;

        leftFrontTarget = (int) (driveRotation + strafeRotation + turnTICKS);
        leftBackTarget = (int) (driveRotation - strafeRotation + turnTICKS);
        rightFrontTarget = (int) (driveRotation - strafeRotation - turnTICKS);
        rightBackTarget = (int) (driveRotation +  strafeRotation - turnTICKS);

        leftFrontDrive.setTargetPosition(leftFrontDrive.getCurrentPosition() + leftFrontTarget);
        leftBackDrive.setTargetPosition(leftBackDrive.getCurrentPosition() + leftBackTarget);
        rightFrontDrive.setTargetPosition(rightFrontDrive.getCurrentPosition() + rightFrontTarget);
        rightBackDrive.setTargetPosition(rightBackDrive.getCurrentPosition() + rightBackTarget);
    }

    public double turnDirection(double angle, boolean returnPower) { // put the ACTUAL angle you want to turn to here. Use this func to set the turn power
        heading = imu.getRobotYawPitchRollAngles().getYaw();
        double goal = angle - heading;
        if (goal > 180) {
            goal -= 360;
        }
        if (goal < -180) {
            goal += 360;
        }

        if (!returnPower) { return goal;}
        if (heading != goal) {
            return (goal / Math.abs(goal) * 0.4);
        } else {
            return 0.0;
        }
    }

    public void setClawPosition(statesOfBeing pinch, statesOfBeing yaw, statesOfBeing axial) {

        //noinspection StatementWithEmptyBody
        if (pinch == pass){ // here to make sure it doesn't waste processing power

        } else if (pinch == enable) { clawPinch.setPosition(CLAW_CLOSE); // Closes Claw:

        } else if (pinch == disable) { clawPinch.setPosition(CLAW_OPEN); } // Opens Claw:


        //noinspection StatementWithEmptyBody
        if (axial == pass) { // here to make sure it doesn't waste processing power

        } else if (axial == enable) { clawAxial.setPosition(CLAW_UP); // Raises Claw:

        } else if (axial == disable) { clawAxial.setPosition(CLAW_COLLAPSED); // Lowers Claw:

        } else if (axial == superposition) { clawAxial.setPosition(CLAW_MID); // Parallels Claw to floor:

        }

        //noinspection StatementWithEmptyBody
        if (yaw == pass) { // here to make sure it doesn't waste processing power

        } else if (yaw == enable) { clawAxial.setPosition(YAW_LEFT); // Raises Claw:

        } else if (yaw == disable) { clawAxial.setPosition(YAW_RIGHT); // Lowers Claw:

        } else if (yaw == superposition) { clawAxial.setPosition(YAW_MID); // Parallels Claw to floor:

        }

    }
    // to stop the claw from switching back and forth while in perpendicular position
        /* more specifically, it stops the elbow from having a preferred side to turn to when at 149 deg
            for ex. Without this, if you went to the perpendicular position from the backwards side,
            it would automatically face the claw the other way
         */
    statesOfBeing elbowDirection = enable;
    public void calibrateClaw(double orientation) { // VERY IMPORTANT: moves the claw to face parallel/perpendicular to the ground
                                                    // relative to the elbow's position
        // returns the current angle without the offset
        double elbowDegTRUE = Math.round((elbowDrive.getCurrentPosition() / ARM_COUNTS_PER_DEGREE));
        double extensionRevs = Math.round((extensionDrive.getCurrentPosition() / EXTENSION_COUNTS_PER_REV) / 0.1) *0.1;
        double targetClawPos; // used to avoid changing the claw position too many times in here

        if (elbowDegTRUE > ELBOW_TRUE_OFFSET) { // sets function to normal/forwards facing FIXME
            targetClawPos = ((-0.17 / 45) * Math.abs(elbowDegTRUE - ELBOW_TRUE_OFFSET) + 0.85); // this if for when the elbow is normal
            elbowDirection = enable; // elbow is facing forward
        } else if (elbowDegTRUE < ELBOW_TRUE_OFFSET){ // changes function to inverse/backwards facing FIXME
            targetClawPos = ((0.16 / 45) * Math.abs(elbowDegTRUE - ELBOW_TRUE_OFFSET) + 0.19); // this is for when the elbow is backwards
            elbowDirection = disable; // elbow is facing backwards
        } else { // for increased accuracy, set to 90 base floor
            if (elbowDirection == enable) {
                targetClawPos = (0.85);
            } else {
                targetClawPos = (0.19);
            }
        }

        /*
        the number next to elbowDegTRUE is at what position the claw should be at your limit
        ex. If the claw is down, the angle should be where the arm is angled at for the claw to be facing straight
        at the floor.
         */
        if (orientation == ELBOW_PERPENDICULAR){ // change position to perpendicular to floor
            if (elbowDegTRUE < 267.00 && elbowDegTRUE > 210.00) { // sets limit between 84 deg from collapsed and 27 deg from collapsed
                targetClawPos = ((-0.17 / 45) * Math.abs(elbowDegTRUE - 207) + CLAW_UP); // this if for when the elbow is normal
                elbowDirection = enable; // elbow is facing forward
            } else if ((elbowDegTRUE >= -0.2 && elbowDegTRUE < 80.00) || (elbowDegTRUE < 80.00 && extensionRevs >= 2.5)){ // sets limit between 214 deg from collapsed and 271 deg from collapsed
                targetClawPos = ((0.15 / 45) * Math.abs(elbowDegTRUE - 77) + CLAW_COLLAPSED); // this is for when the elbow is backwards
                elbowDirection = disable; // elbow is facing backwards
            }
        }
        if (elbowDegTRUE == 0){targetClawPos = 0.75;} // pass submersible clearance
        clawAxial.setPosition(targetClawPos);
    }

    /**
     * Drives the extender during teleOp
     *
     * @param input Dictates which direction the extender will move. Intended to be used as a gamepad stick.
     */
    public int driveExtenderPosition(double input){
        input = Range.clip(input, -1, 1);
        // make sure input doesn't exceed the possible power for the motor

        extensionDrive.setPower(input);

        if (input > 0){
            // if input is positive, go to max pos
            return (int) EXTENSION_MAXIMUM_COUNT;
        } else if (input < 0){
            // if input is negative, go to min pos
            return 0;
        } else {
            // if input is 0, stop at where you're at
            return extensionDrive.getCurrentPosition();
        }
    }

    /**
     *
     * @param robotPos current position of robot
     * @param heading either IMU heading or LL heading
     * @return returns a counter value for the elbow to face the high rung
     */
    public int elbowTrigPosition(Pose3D robotPos, double heading){
        Position holder = robotPos.getPosition();
        Position pendingPos = new Position(DistanceUnit.INCH, 0, Math.abs(holder.y)*MtoIN, (holder.z*MtoIN) + 5, 0);
        // pythagorean theorem ( letters correspond to the triangle part )
        double distanceA = pendingPos.y - 24;
        double heightB = 27 - pendingPos.z;
        double distanceC = Math.sqrt((distanceA*distanceA) + (heightB*heightB));
        double cosOfAngle = distanceA/distanceC;
        double angle = Math.toDegrees(Math.acos(cosOfAngle));
        myOpMode.telemetry.addData(String.valueOf(angle), "");
        // check if you're facing forward
        if ((heading + 90) >= 0) {
            // return angle for a forward facing elbow
            return (int) ((angle + ELBOW_ANGLE_OFFSET) * ARM_COUNTS_PER_DEGREE);
        } else {
            // return angle for a rearward facing elbow
            return (int) ((ELBOW_BACKWARD_PARALLEL - angle) * ARM_COUNTS_PER_DEGREE);
        }
    }

    public void extensionBoundBox(){
        // cos (angle) = A/C ; targetC = flatA/ cos(angle) : flatA is 17, 25, 2.4
        // 17 - 2.4 = 14.6
        // u rike my mat eh?
        double cosAngle = Math.cos(   Math.toRadians( (elbowDrive.getCurrentPosition()/ARM_COUNTS_PER_DEGREE) - 37)   );
        double targetC = 17 /  cosAngle;
        double ticksC = (targetC - 14.6)*EXTENSION_COUNTS_PER_INCH;

        if (extensionDrive.getCurrentPosition() >= ticksC) {
            extensionDrive.setTargetPosition( (int) (ticksC));
        }
    }

    public void velocityElbowHandler(double armDegC){
        elbowDrive.setTargetPosition((int) (armDegC));
        ((DcMotorEx) elbowDrive).setVelocity(2200);
        elbowDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void velocityExtensionHandler(double inchesC){
        extensionDrive.setTargetPosition((int) (inchesC));
        ((DcMotorEx) extensionDrive).setVelocity(2850);
        extensionDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}