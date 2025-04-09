package org.firstinspires.ftc.team00000.v2;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.team00000.v2.vision.ColorVisionSubsystem;

public class RobotHardware {

    // Declare OpMode members
    private final LinearOpMode myOpMode; // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects (Make them private so that they CANT be accessed externally)
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    public DcMotor shoulderDrive = null;
    public DcMotor armDrive = null;
    private Servo wristDrive = null;
    private Servo clawDrive = null;
    public ColorVisionSubsystem colorVision;

    // Define Sensor objects (Make them private so that they CANT be accessed externally)
    private IMU imu = null; // Universal IMU interface
    private double headingError;

    /*
    These variables are declared here (as class members) so they can be updated in various methods, but still be
    displayed by sendTelemetry()
     */

    public double leftFrontPower;
    public double leftBackPower;
    public double rightFrontPower;
    public double rightBackPower;

    public double DRIVE_SPEED;
    public double TURN_SPEED;
    public double HEADING_THRESHOLD;

    public double COUNTS_PER_MOTOR_REV;
    public double DRIVE_GEAR_REDUCTION;
    public double WHEEL_DIAMETER_INCHES;
    public double COUNTS_PER_INCH;

    public double P_TURN_GAIN;
    public double P_DRIVE_GAIN;

    public double SHOULDER_TICKS_PER_DEGREE;
    public double ARM_TICKS_PER_REV;
    public double TOLERANCE_TICKS;

    public double SHOULDER_COLLAPSED_INTO_ROBOT;
    public double SHOULDER_SAMPLE_RETRACTED;
    public double SHOULDER_WINCH_ROBOT;
    public double SHOULDER_SAMPLE_EXTENDED;
    public double SHOULDER_LOW_CHAMBER;
    public double SHOULDER_LOW_BUCKET;
    public double SHOULDER_HIGH_CHAMBER;
    public double SHOULDER_HIGH_BUCKET;
    public double SHOULDER_ATTACH_HANGING_HOOK;

    public double ARM_RETRACTED;
    public double ARM_HIGH_CHAMBER;
    public double ARM_EXTENDED;

    public double WRIST_STRAIGHT;
    public double WRIST_45;
    public double WRIST_ROTATE;
    public double WRIST_FLIP;

    public double CLAW_OPEN;
    public double CLAW_CLOSE;

    public double shoulderPosition;
    public double armPosition;

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
    public void init(){

        // Define and initialize ALL installed motors (note: need to use reference to the actual OpMode).
        leftFrontDrive = myOpMode.hardwareMap.get(DcMotor.class,"left_front_drive");
        leftBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_back_drive");
        shoulderDrive = myOpMode.hardwareMap.get(DcMotor.class, "shoulder_drive");
        armDrive = myOpMode.hardwareMap.get(DcMotor.class, "arm_drive");
        wristDrive = myOpMode.hardwareMap.get(Servo.class, "wrist_drive");
        clawDrive = myOpMode.hardwareMap.get(Servo.class, "claw_drive");

        /*
        These constants define the desired driving/control characteristics. They can/should be tweaked to suit the specific
        robot drive train.
         */
        DRIVE_SPEED = 0.4; // Maximum autonomous driving speed for better distance accuracy.
        TURN_SPEED = 0.2; // Maximum autonomous turning speed for better rotational accuracy.
        HEADING_THRESHOLD = 1.0; // How close must the heading get to the target before moving to next step.
        // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.

        /*
        Define the Proportional control coefficient (or GAIN) for "heading control". We define one value when Turning
        (large errors), another when Strafing (medium errors), and the other is used when Driving straight (small errors).
        Increase these numbers if the heading does not correct strongly enough (e.g. a heavy robot or using tracks).
        Decrease these numbers if the heading does not settle on the correct value (e.g. very agile robot with omni wheels).
         */
        P_TURN_GAIN = 0.02; // Larger is more responsive, but also less stable.
        P_DRIVE_GAIN = 0.03; // Larger is more responsive, but also less stable.

        /*
        Calculate the COUNTS_PER_INCH for your specific drive train. Go to your motor vendor website to determine your
        motor's COUNT_PER_MOTOR_REV. For external drive gearing set DRIVE_GEAR_REDUCTION as needed. For example, use a
        value of 2.0 for a 12-tooth spur driving a 24-tooth spur gear. This is gearing DOWN for less speed and more
        torque. For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of the wheel
        rotation.
         */
        COUNTS_PER_MOTOR_REV = 537.7; // goBILDA
        DRIVE_GEAR_REDUCTION =  1.0; // No external gearing
        WHEEL_DIAMETER_INCHES = 3.77953; // goBILDA (96mm Diameter)
        COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

        /*
        28: (number of encoder ticks per rotation of the bare motor)
        50.9: (exact gear ratio of the 60:1 NeveRest gearbox)
        100.0/20/0: This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
        1.0 / 360.0 (ticks per degree, not per rotation)
        */
        SHOULDER_TICKS_PER_DEGREE = 28 * 50.9 * 100.0 / 20.0 / 360.0;
        ARM_TICKS_PER_REV = 28 * 19.2 * 1.0;
        TOLERANCE_TICKS = 10.0;

        /*
        These constants hold the position that the arm is commanded to run to and are relative to where the arm was
        located when you start the OpMode. In these variables you will see a number of degrees, multiplied by the ticks
        per degree of the arm. This results in the number of encoders ticks the arm needs to move to achieve the ideal
        set position of the arm.
        */
        SHOULDER_COLLAPSED_INTO_ROBOT = 0 * SHOULDER_TICKS_PER_DEGREE;
        SHOULDER_SAMPLE_RETRACTED = 11 * SHOULDER_TICKS_PER_DEGREE;
        SHOULDER_WINCH_ROBOT =  15 * SHOULDER_TICKS_PER_DEGREE;
        SHOULDER_SAMPLE_EXTENDED = 23 * SHOULDER_TICKS_PER_DEGREE;
        SHOULDER_LOW_CHAMBER = 55 * SHOULDER_TICKS_PER_DEGREE;
        SHOULDER_LOW_BUCKET = 83  * SHOULDER_TICKS_PER_DEGREE;
        SHOULDER_HIGH_CHAMBER = 95 * SHOULDER_TICKS_PER_DEGREE;
        SHOULDER_HIGH_BUCKET = 102 * SHOULDER_TICKS_PER_DEGREE;
        SHOULDER_ATTACH_HANGING_HOOK =  130 * SHOULDER_TICKS_PER_DEGREE;

        ARM_RETRACTED = 0 * ARM_TICKS_PER_REV;
        ARM_HIGH_CHAMBER = 1.4 * ARM_TICKS_PER_REV;
        ARM_EXTENDED = 4 * ARM_TICKS_PER_REV;

        WRIST_STRAIGHT = 0.05;
        WRIST_45 = 0.25;
        WRIST_ROTATE = 0.5;
        WRIST_FLIP = 1.0;

        CLAW_CLOSE = 0.99;
        CLAW_OPEN = 0.5;

        shoulderPosition = 10;
        armPosition = 0;

        /*
         Define how the hub is mounted on the robot to get the correct Yaw, Pitch, and Roll values. There are two input
         parameters required to fully specify the orientation. (1) the first parameter specifies the direction of the
         printed logo on the hub is pointing. (2) the second parameter specifies the direction the USB connector on the
         hub is pointing. All directions are relative to the robot, and left/right is as-viewed from behind the robot.
         */
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));

        imu = myOpMode.hardwareMap.get(IMU.class, "imu");
        imu.initialize(parameters);

        WebcamName webcam = myOpMode.hardwareMap.get(WebcamName.class, "Webcam 1");
        colorVision = new ColorVisionSubsystem(webcam);

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
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        shoulderDrive.setDirection(DcMotor.Direction.FORWARD);
        armDrive.setDirection(DcMotor.Direction.REVERSE);

        // Ensure the robot is stationary. Reset the encoders and set the motors to BREAK mode
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shoulderDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set the encoders for closed loop speed control, and reset the heading.
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set the TargetPosition to 0. Then we'll set the RunMode to RUN_TO_POSITION, and we'll ask it to stop and reset.
        shoulderDrive.setTargetPosition(0);
        armDrive.setTargetPosition(0);
        shoulderDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shoulderDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Reset the IMU when initializing the hardware class
        imu.resetYaw();

        // Make sure that the claw is open and the wrist is
        setClawPosition(CLAW_OPEN);
        setWristPosition(WRIST_STRAIGHT);
        setArmPosition(ARM_RETRACTED);
        setShoulderPosition(SHOULDER_COLLAPSED_INTO_ROBOT);

        // Wait for the game to start
        while (myOpMode.opModeInInit()) {
            myOpMode.telemetry.addData("Status", "Hardware Initialized");
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
    public void driveRobotCentric(double drive, double strafe, double turn) {

        double max;

        /*
         Combine the joystick requests for each axis-motion to determine each wheel's power. Set up a variable for each
         drive wheel to save the power level for telemetry. Denominator is the largest motor power (absolute value) or
         1. This ensures all the powers maintain the same ratio, but only when at least one is out of the range [-1, 1]
         */
        double denominator = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(turn), 1);
        leftFrontPower = (drive + strafe + turn) / denominator;
        leftBackPower = (drive - strafe + turn) / denominator;
        rightFrontPower = (drive - strafe - turn) / denominator;
        rightBackPower = (drive + strafe - turn) / denominator;

        // Normalize the values so no wheel power exceeds 100%.
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

        // Use existing function to drive all wheels.
        setDrivePower(leftFrontPower, leftBackPower, rightFrontPower, rightBackPower);
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
        rightBackPower = (driveRotation + strafeRotation - turn) / denominator;

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

    /**
     * Pass the requested wheel motor power to the appropriate hardware drive motors.
     *
     * @param leftFrontWheel    Forward/Reverse driving power (-1.0 to 1.0) +ve is forward
     * @param leftBackWheel    Forward/Reverse driving power (-1.0 to 1.0) +ve is forward
     * @param rightFrontWheel   Forward/Reverse driving power (-1.0 to 1.0) +ve is forward
     * @param rightBackWheel   Forward/Reverse driving power (-1.0 to 1.0) +ve is forward
     */
    public void setDrivePower(double leftFrontWheel, double leftBackWheel, double rightFrontWheel, double rightBackWheel) {
        //Output the values to the motor drives.
        leftFrontDrive.setPower(leftFrontWheel);
        leftBackDrive.setPower(leftBackWheel);
        rightFrontDrive.setPower(rightFrontWheel);
        rightBackDrive.setPower(rightBackWheel);
    }

    // ******************** Articulation control functions ********************

    /**
     * set the target position of our arm to match the variables that was selected by the driver. We also set the target
     * velocity (speed) the motor runs at, and use setMode to run it.
     */
    public void setShoulderPosition(double position) {
        shoulderDrive.setTargetPosition((int) (position));
        ((DcMotorEx) shoulderDrive).setVelocity(2100);
        shoulderDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * set the target position of our arm to match the variables that was selected by the driver. We also set the target
     * velocity (speed) the motor runs at, and use setMode to run it.
     */
    public void setArmPosition(double position) {
        armDrive.setTargetPosition((int) (position));
        ((DcMotorEx) armDrive).setVelocity(2100);
        armDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Send the wrist-servo to position
     *
     * @param position  (0.0 to 1.0) +ve is extended
     */
    public void setWristPosition(double position) {
        wristDrive.setPosition(position);
    }

    /**
     * Send the claw-servo to position
     *
     * @param position  (0.0 to 1.0) +ve is open
     */
    public void setClawPosition(double position) {
        clawDrive.setPosition(position);
    }

    // ******************** RoadRunner control functions ********************

    public class MoveShoulder implements Action {
        private final double position;

        public MoveShoulder(double position) {
            this.position = position;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            setShoulderPosition(position);
            return Math.abs(shoulderDrive.getCurrentPosition() - position) > TOLERANCE_TICKS;
        }
    }

    public Action moveShoulder(double position) {
        return new MoveShoulder(position);
    }

    public class MoveArm implements Action {
        private final double position;

        public MoveArm(double position) {
            this.position = position;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            setArmPosition(position);
            return Math.abs(armDrive.getCurrentPosition() - position) > TOLERANCE_TICKS;
        }
    }

    public Action moveArm(double position) {
        return new MoveArm(position);
    }

    public class MoveWrist implements Action {
        private final double position;

        public MoveWrist(double position) {
            this.position = position;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            setWristPosition(position);
            return false;  // Servo movements are instantaneous, so return false
        }
    }

    public Action moveWrist(double position) {
        return new MoveWrist(position);
    }

    public class MoveClaw implements Action {
        private final double position;

        public MoveClaw(double position) {
            this.position = position;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            setClawPosition(position);
            return false;  // Servo movements are instantaneous, so return false
        }
    }

    public Action moveClaw(double position) {
        return new MoveClaw(position);
    }

    // ******************** Vision control functions ********************

}