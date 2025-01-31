package org.firstinspires.ftc.team13588;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class RobotHardware {

    // Declare OpMode members
    private final LinearOpMode myOpMode; // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects (Make them private so that they CANT be accessed externally)
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    public DcMotor shoulderDrive = null;
    public DcMotor armDrive = null;
    private Servo wristDrive = null;
    private Servo clawDrive = null;

    // Define Sensor objects (Make them private so that they CANT be accessed externally)
    private IMU imu = null; // Universal IMU interface
    private double headingError;

    /*
    These variables are declared here (as class members) so they can be updated in various methods, but still be
    displayed by sendTelemetry()
     */
    private double targetHeading;
    private double driveSpeed;
    private double turnSpeed;
    public double leftPower;
    public double rightPower;
    public int leftTarget;
    public int rightTarget;

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
    public double SHOULDER_LOW_RUNG;
    public double SHOULDER_LOW_BUCKET;
    public double SHOULDER_HIGH_RUNG;
    public double SHOULDER_HIGH_BUCKET;
    public double SHOULDER_ATTACH_HANGING_HOOK;
    public double SHOULDER_WINCH_ROBOT;
    public double ARM_RETRACTED;
    public double WRIST_STRAIGHT;
    public double WRIST_ROTATE;
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
        leftDrive = myOpMode.hardwareMap.get(DcMotor.class,"left_drive");
        rightDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_drive");
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
        WHEEL_DIAMETER_INCHES = 3.77953; // goBILDA 3601 Series Rhino Wheel (96mm Diameter)
        COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

        /*
        28: (number of encoder ticks per rotation of the bare motor)
        50.9: (exact gear ratio of the 60:1 NeveRest gearbox)
        100.0/20/0: This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
        1.0 / 360.0 (ticks per degree, not per rotation)
        */
        SHOULDER_TICKS_PER_DEGREE = 28 * 50.9 * 100.0/20.0 * 1.0 / 360.0;
        ARM_TICKS_PER_REV = 28 * 50.9 * 1.0;
        TOLERANCE_TICKS = 10.0;

        /*
        These constants hold the position that the arm is commanded to run to and are relative to where the arm was
        located when you start the OpMode. In these variables you will see a number of degrees, multiplied by the ticks
        per degree of the arm. This results in the number of encoders ticks the arm needs to move to achieve the ideal
        set position of the arm.
        */
        SHOULDER_COLLAPSED_INTO_ROBOT = 0 * SHOULDER_TICKS_PER_DEGREE;
        SHOULDER_LOW_RUNG = 55 * SHOULDER_TICKS_PER_DEGREE;
        SHOULDER_LOW_BUCKET = 83  * SHOULDER_TICKS_PER_DEGREE;
        SHOULDER_HIGH_RUNG = 85 * SHOULDER_TICKS_PER_DEGREE;
        SHOULDER_HIGH_BUCKET = 102 * SHOULDER_TICKS_PER_DEGREE;
        SHOULDER_ATTACH_HANGING_HOOK =  130 * SHOULDER_TICKS_PER_DEGREE;
        SHOULDER_WINCH_ROBOT =  15 * SHOULDER_TICKS_PER_DEGREE;

        ARM_RETRACTED = 0 * ARM_TICKS_PER_REV;

        WRIST_STRAIGHT = 0.5;
        WRIST_ROTATE = 1.0;

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

        /*
         Most robots need the motors on one side to be reversed to drive forward. The motor reversals shown here are
         for a "direct drive" robot (the wheels turn in the same direction as the motor shaft). If your robot has
         additional gear reductions or uses a right-angled drive, it is important to ensure that your motors are turning
         in the correct direction. So, start out with the reversals here, BUT when you first test your robot, push the
         left joystick forward and observe the wheels turn. Reverse the direction (flip FORWARD <-> REVERSE) of any
         wheel that runs backward. Keep testing until ALL the wheels move the robot forward when you push the left
         joystick forward.
         */
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        shoulderDrive.setDirection(DcMotor.Direction.FORWARD);
        armDrive.setDirection(DcMotor.Direction.REVERSE);

        // Ensure the robot is stationary. Reset the encoders and set the motors to BREAK mode
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shoulderDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set the encoders for closed loop speed control, and reset the heading.
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
            myOpMode.telemetry.addData("Heading", "%4.0f", getHeading());
            myOpMode.telemetry.update();
        }
    }

    // ******************** HIGH Level driving functions ********************

    /**
     * Drive on a fixed compass heading (angle), based on encoder counts. Move will stop if either of the following
     * conditions occurs: 1) Move gets to the desired position. 2) Driver stops the OpMode
     *
     * @param maxDriveSpeed MAX Speed for forward/rev motion (range 0 to +1.0).
     * @param distance      Distance (in inches) to move from current position. Negative distance means move backward.
     * @param heading       Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                      0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                      If a relative angle is required, add/subtract from current heading.
     */
    public void driveStraight(double maxDriveSpeed, double distance, double heading) {

        // Ensure that the OpMode is still active
        if (myOpMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int)(distance * COUNTS_PER_INCH);
            leftTarget = leftDrive.getCurrentPosition() + moveCounts;
            rightTarget = rightDrive.getCurrentPosition() + moveCounts;

            // Set Target FIRST, then turn on RUN_TO_POSITION
            leftDrive.setTargetPosition(leftTarget);
            rightDrive.setTargetPosition(rightTarget);

            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            driveRobotCentric(maxDriveSpeed, 0);

            /*
            Keep looping while we are still active, and there is time left, and all motors are running. Note, We use
            (isBusy() && isBusy() && isBusy()) in the loop test, which means that when ANY motor hits its target
            position, the motion will stop. This is "safer" in the event that the robot will always end the motion as
            soon as possible. However, if you require that ALL motors have finished their moves before the robot
            continues onto the next step, use (isBusy() || isBusy() || isBusy()) in the loop test.
             */
            while (myOpMode.opModeIsActive() && (leftDrive.isBusy() && rightDrive.isBusy())) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving power.
                driveRobotCentric(driveSpeed, turnSpeed);

                // Display drive status for the driver.
                sendTelemetry(true);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            driveRobotCentric(0, 0);
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Spin on the central axis to point in a new direction. Move will stop if either of these conditions occur:
     * 1) Move gets to the heading (angle), or 2) Driver stops the OpMode running.
     *
     * @param maxTurnSpeed   Desired MAX speed of turn. (range 0 to +1.0)
     * @param heading       Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                      0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                      If a relative angle is required, add/subtract from current heading.
     *
     */
    public void turnToHeading(double maxTurnSpeed, double heading) {

        // Run getSteeringCorrection() once to  pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // Keep looping while we are still active, and not on heading.
        while (myOpMode.opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            driveRobotCentric(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion
        driveRobotCentric(0, 0);
    }

    /**
     *  Obtain & hold a heading for a finite amount of time. Move will stop once the requested time has elapsed.
     *  This function is useful for giving the robot a moment to stabilize its heading between movements.
     *
     * @param maxTurnSpeed   Maximum differential turn speed (range 0 to +1.0)
     * @param heading       Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                      0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                      If a relative angle is required, add/subtract from current heading.
     * @param holdTime      Length of time (in seconds) to hold the specified heading.
     */
    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        // keep looping while we have time remaining.
        while (myOpMode.opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            driveRobotCentric(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        driveRobotCentric(0, 0);
    }

    // ******************** LOW Level driving functions ********************

    /**
     * Use a Proportional Controller to determine how much steering correction is required.
     *
     * @param desiredHeading        The desired absolute heading (relative to last heading reset)
     * @param proportionalGain      Gain factor applied to heading error to obtain turning power.
     * @return                      Turning power needed to get to required heading.
     */
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Determine the heading current error
        headingError = targetHeading - getHeading();

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    /**
     * Calculate the motor powers required to achieve the requested robot motions:
     * Drive (Axial motion), Strafe (Lateral motion), and Turn (Yaw motion)
     * Then send these power levels to the motors.
     *
     * @param drive     Forward/Reverse driving power (-1.0 to 1.0) +ve is forward
     * @param turn      Right/Left turning power (-1.0 to 1.0) +ve is clockwise
     */
    public void driveRobotCentric(double drive, double turn) {

        double max;

        /*
         Combine the joystick requests for each axis-motion to determine each wheel's power. Set up a variable for each
         drive wheel to save the power level for telemetry. Denominator is the largest motor power (absolute value) or
         1. This ensures all the powers maintain the same ratio, but only when at least one is out of the range [-1, 1]
         */
        double denominator = Math.max(Math.abs(drive) + Math.abs(turn), 1);
        leftPower = (drive + turn) / denominator;
        rightPower = (drive - turn) / denominator;

        // Normalize the values so no wheel power exceeds 100%.
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftPower), Math.abs(rightPower));

        if (max > 1.0) {
            leftPower /= max;
            rightPower /= max;
        }

        // Use existing function to drive all wheels.
        setDrivePower(leftPower, rightPower);
    }

    /**
     * Pass the requested wheel motor power to the appropriate hardware drive motors.
     *
     * @param leftWheel    Forward/Reverse driving power (-1.0 to 1.0) +ve is forward
     * @param rightWheel   Forward/Reverse driving power (-1.0 to 1.0) +ve is forward
     */
    public void setDrivePower(double leftWheel, double rightWheel) {
        //Output the values to the motor drives.
        leftDrive.setPower(leftWheel);
        rightDrive.setPower(rightWheel);
    }

    /**
     *  Display the various control parameters while driving
     *
     * @param straight     Set to true if we are driving straight, and the encoder positions should be included
     *                  in the telemetry.
     */
    private void sendTelemetry(boolean straight) {

        if (straight) {
            myOpMode.telemetry.addData("Motion", "Drive Straight");
            myOpMode.telemetry.addData("Target Pos L:R", "%7d:%7d",
                    leftTarget, rightTarget);
            myOpMode.telemetry.addData("Actual Pos L:R", "%7d:%7d",
                    leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
        } else {
            myOpMode.telemetry.addData("Motion", "Turning");
        }

        myOpMode.telemetry.addData("Heading- Target : Current", "%5.2f : %5.0f", targetHeading, getHeading());
        myOpMode.telemetry.addData("Error  : Steer Pwr",  "%5.1f : %5.1f", headingError, turnSpeed);
        myOpMode.telemetry.addData("Wheel Speeds L : R", "%5.2f : %5.2f : %5.2f : %5.2f", leftPower, rightPower);
        myOpMode.telemetry.update();
    }

    /**
     * read the Robot heading directly from the IMU (in degrees)
     */
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
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
        armDrive.setTargetPosition((int) (4 * position * ARM_TICKS_PER_REV));
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
}