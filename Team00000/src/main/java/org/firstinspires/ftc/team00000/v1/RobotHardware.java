package org.firstinspires.ftc.team00000.v1;

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
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    // private DcMotor leftVerticalArmDrive = null;
    private DcMotor rightVerticalArmDrive = null;
    private Servo verticalWristDrive = null;
    private Servo verticalClawDrive = null;
    private Servo leftHorizontalArmDrive = null;
    private Servo rightHorizontalArmDrive = null;
    private Servo horizontalWristDrive = null;
    private Servo horizontalClawDrive = null;

    // Define Sensor objects (Make them private so that they CANT be accessed externally)
    private IMU imu = null; // Universal IMU interface
    private double headingError;

    /*
    These variables are declared here (as class members) so they can be updated in various methods, but still be
    displayed by sendTelemetry()
     */
    private double targetHeading;
    public double axialPower;
    public double lateralPower;
    public double yawPower;
    public double leftFrontPower;
    public double leftBackPower;
    public double rightFrontPower;
    public double rightBackPower;
    public int leftFrontTarget;
    public int leftBackTarget;
    public int rightFrontTarget;
    public int rightBackTarget;

    public double DRIVE_SPEED;
    public double STRAFE_SPEED;
    public double TURN_SPEED;
    public double HEADING_THRESHOLD;

    public double COUNTS_PER_MOTOR_REV;
    public double DRIVE_GEAR_REDUCTION;
    public double WHEEL_DIAMETER_INCHES;
    public double COUNTS_PER_INCH;

    public double P_YAW_GAIN;
    public double P_LATERAL_GAIN;
    public double P_AXIAL_GAIN;

    public double ARM_TICKS_PER_DEGREE;
    public double TOLERANCE_TICKS;

    public double VERTICAL_ARM_MIN;
    public double VERTICAL_ARM_LOW_BASKET;
    public double VERTICAL_ARM_HIGH_BASKET;
    public double VERTICAL_ARM_HIGH_RUNG;

    public double HORIZONTAL_ARM_MIN;
    public double HORIZONTAL_ARM_MAX;

    public double VERTICAL_WRIST_TRANSFER;
    public double VERTICAL_WRIST_RUNG;
    public double VERTICAL_WRIST_BUCKET;
    public double VERTICAL_WRIST_DROP;

    public double HORIZONTAL_WRIST_TRANSFER;
    public double HORIZONTAL_WRIST_WALL;
    public double HORIZONTAL_WRIST_PICKUP;

    public double WRIST_FUDGE;

    public double CLAW_OPEN;
    public double CLAW_CLOSE;

    public double verticalArmPosition;

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
        // leftVerticalArmDrive = myOpMode.hardwareMap.get(DcMotor.class, "left_vertical_arm_drive");
        rightVerticalArmDrive = myOpMode.hardwareMap.get(DcMotor. class, "right_vertical_arm_drive");
        verticalWristDrive = myOpMode.hardwareMap.get(Servo.class, "vertical_wrist_drive");
        verticalClawDrive = myOpMode.hardwareMap.get(Servo.class, "vertical_claw_drive");
        leftHorizontalArmDrive = myOpMode.hardwareMap.get(Servo.class, "left_horizontal_arm_drive");
        rightHorizontalArmDrive = myOpMode.hardwareMap.get(Servo.class, "right_horizontal_arm_drive");
        horizontalWristDrive = myOpMode.hardwareMap.get(Servo.class, "horizontal_wrist_drive");
        horizontalClawDrive = myOpMode.hardwareMap.get(Servo.class, "horizontal_claw_drive");

        /*
        These constants define the desired driving/control characteristics. They can/should be tweaked to suit the specific
        robot drive train.
         */
        DRIVE_SPEED = 0.7; // Maximum autonomous driving speed for better distance accuracy.
        STRAFE_SPEED = 0.7; // Maximum autonomous strafing speed for better distance accuracy.
        TURN_SPEED = 0.7; // Maximum autonomous turning speed for better rotational accuracy.
        HEADING_THRESHOLD = 0.1; // How close must the heading get to the target before moving to next step.
        // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.

        /*
        Define the Proportional control coefficient (or GAIN) for "heading control". We define one value when Turning
        (large errors), another when Strafing (medium errors), and the other is used when Driving straight (small errors).
        Increase these numbers if the heading does not correct strongly enough (e.g. a heavy robot or using tracks).
        Decrease these numbers if the heading does not settle on the correct value (e.g. very agile robot with omni wheels).
         */
        P_YAW_GAIN = 0.01; // Larger is more responsive, but also less stable.
        P_LATERAL_GAIN = 0.03; // Larger is more responsive, but also less stable.
        P_AXIAL_GAIN = 0.03; // Larger is more responsive, but also less stable.

        /*
        Calculate the COUNTS_PER_INCH for your specific drive train. Go to your motor vendor website to determine your
        motor's COUNT_PER_MOTOR_REV. For external drive gearing set DRIVE_GEAR_REDUCTION as needed. For example, use a
        value of 2.0 for a 12-tooth spur driving a 24-tooth spur gear. This is gearing DOWN for less speed and more
        torque. For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of the wheel
        rotation.
         */
        COUNTS_PER_MOTOR_REV = 537.7; // goBILDA
        DRIVE_GEAR_REDUCTION =  1.0; // No external gearing
        WHEEL_DIAMETER_INCHES = 4.0945; // goBILDA 104 mm
        COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

        /*
        28: (number of encoder ticks per rotation of the bare motor)
        19.2: (exact gear ratio of the 60:1 NeveRest gearbox)
        1.0: (no external gear reduction)
        1.0 / 360.0 (ticks per degree, not per rotation)
        */
        ARM_TICKS_PER_DEGREE = 28 * 19.2 * 1.0 * 1.0 / 360.0;
        TOLERANCE_TICKS = 10.0;

        /*
        These constants hold the position that the arm is commanded to run to and are relative to where the arm was
        located when you start the OpMode. In these variables you will see a number of degrees, multiplied by the ticks
        per degree of the arm. This results in the number of encoders ticks the arm needs to move to achieve the ideal
        set position of the arm.
        */
        VERTICAL_ARM_MIN = 0 * ARM_TICKS_PER_DEGREE;
        VERTICAL_ARM_LOW_BASKET = 360 * 2 * ARM_TICKS_PER_DEGREE;
        VERTICAL_ARM_HIGH_BASKET = 360 * 5.25 * ARM_TICKS_PER_DEGREE;
        // no vertical accent is needed by the arms to reach the low rung
        VERTICAL_ARM_HIGH_RUNG = 360 * 2.5 * ARM_TICKS_PER_DEGREE;

        HORIZONTAL_ARM_MIN = 0.0;
        HORIZONTAL_ARM_MAX = 1.0;

        VERTICAL_WRIST_TRANSFER = 0.039;
        VERTICAL_WRIST_RUNG = 0.105;
        VERTICAL_WRIST_BUCKET = 0.115;
        VERTICAL_WRIST_DROP = 0.165;

        HORIZONTAL_WRIST_TRANSFER = 0.555;
        HORIZONTAL_WRIST_WALL = 0.290;
        HORIZONTAL_WRIST_PICKUP = 0.0140;

        WRIST_FUDGE = 0.0025;

        CLAW_CLOSE = 1.0;
        CLAW_OPEN = 0.95;

        verticalArmPosition = (int) VERTICAL_ARM_MIN;

        /*
         Define how the hub is mounted on the robot to get the correct Yaw, Pitch, and Roll values. There are two input
         parameters required to fully specify the orientation. (1) the first parameter specifies the direction of the
         printed logo on the hub is pointing. (2) the second parameter specifies the direction the USB connector on the
         hub is pointing. All directions are relative to the robot, and left/right is as-viewed from behind the robot.
         */
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));

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
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        // leftVerticalArmDrive.setDirection(DcMotor.Direction.REVERSE);
        rightVerticalArmDrive.setDirection(DcMotor.Direction.FORWARD);

        // Ensure the robot is stationary. Reset the encoders and set the motors to BREAK mode
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // leftVerticalArmDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightVerticalArmDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set the encoders for closed loop speed control, and reset the heading.
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set the TargetPosition to 0. Then we'll set the RunMode to RUN_TO_POSITION, and we'll ask it to stop and reset.
        // leftVerticalArmDrive.setTargetPosition(0);
        rightVerticalArmDrive.setTargetPosition(0);
        // leftVerticalArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightVerticalArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // leftVerticalArmDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightVerticalArmDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Reset the IMU when initializing the hardware class
        imu.resetYaw();

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
            leftFrontTarget = leftFrontDrive.getCurrentPosition() + moveCounts;
            leftBackTarget = leftBackDrive.getCurrentPosition() + moveCounts;
            rightFrontTarget = rightFrontDrive.getCurrentPosition() + moveCounts;
            rightBackTarget = rightBackDrive.getCurrentPosition() + moveCounts;

            // Set Target FIRST, then turn on RUN_TO_POSITION
            leftFrontDrive.setTargetPosition(leftFrontTarget);
            leftBackDrive.setTargetPosition(leftBackTarget);
            rightFrontDrive.setTargetPosition(rightFrontTarget);
            rightBackDrive.setTargetPosition(rightBackTarget);

            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            driveRobotCentric(maxDriveSpeed, 0, 0);

            /*
            Keep looping while we are still active, and there is time left, and all motors are running. Note, We use
            (isBusy() && isBusy() && isBusy()) in the loop test, which means that when ANY motor hits its target
            position, the motion will stop. This is "safer" in the event that the robot will always end the motion as
            soon as possible. However, if you require that ALL motors have finished their moves before the robot
            continues onto the next step, use (isBusy() || isBusy() || isBusy() || isBusy()) in the loop test.
             */
            while (myOpMode.opModeIsActive() &&
                    (leftFrontDrive.isBusy() && leftBackDrive.isBusy() &&
                            rightFrontDrive.isBusy() && rightBackDrive.isBusy())) {

                // Determine required steering to keep on heading
                yawPower = getSteeringCorrection(heading, P_AXIAL_GAIN);
            }

            // if driving in reverse, the motor correction also needs to be reversed
            if (distance < 0)
                yawPower *= -1.0;

            // Apply the turning correction to the current driving power.
            driveRobotCentric(axialPower, 0, yawPower);

            // Display drive status for the driver.
            sendTelemetry(true);

            // Stop all motion & Turn off RUN_TO_POSITION
            driveRobotCentric(0, 0, 0);
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Drive the robot "distanceInches" along "angleDegrees" (robot-centric),
     * while holding an absolute heading of "desiredHeading".
     *
     * @param maxSpeed       Maximum speed (0..1).
     * @param distanceInches Distance in inches to move from current position.
     * @param angleDegrees   Robot-relative travel angle, measured from forward.
     *                       e.g. +90 => left, -90 => right, 0 => forward, etc.
     * @param desiredHeading Absolute heading (in degrees from IMU) to hold.
     */
    public void driveOmni(double maxSpeed, double distanceInches, double angleDegrees, double desiredHeading) {
        // Ensure that the OpMode is still active
        if (myOpMode.opModeIsActive()) {

            // Convert angle to radians
            double angleRadians = Math.toRadians(angleDegrees);

            // Compute the forward/back (axial) and left/right (lateral) distances from the desired distance and angle
            double axialDistance = distanceInches * Math.cos(angleRadians);
            double lateralDistance = distanceInches * Math.sin(angleRadians);

            // Convert those distances to encoder counts
            int axialMoveCounts = (int)(axialDistance * COUNTS_PER_INCH);
            int lateralMoveCounts = (int)(lateralDistance * COUNTS_PER_INCH);

            // Compute new encoder targets for each wheel
            leftFrontTarget = leftFrontDrive.getCurrentPosition() + axialMoveCounts + lateralMoveCounts;
            leftBackTarget = leftBackDrive.getCurrentPosition() + axialMoveCounts - lateralMoveCounts;
            rightFrontTarget = rightFrontDrive.getCurrentPosition() + axialMoveCounts - lateralMoveCounts;
            rightBackTarget = rightBackDrive.getCurrentPosition() + axialMoveCounts + lateralMoveCounts;

            // Set targets and RUN_TO_POSITION on all drive motors
            leftFrontDrive.setTargetPosition(leftFrontTarget);
            leftBackDrive.setTargetPosition(leftBackTarget);
            rightFrontDrive.setTargetPosition(rightFrontTarget);
            rightBackDrive.setTargetPosition(rightBackTarget);

            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the initial driving power. We'll drive in the correct ratio of forward/back vs left/right so that the
            // path is more direct from the start.
            double totalDistance = Math.hypot(axialDistance, lateralDistance);
            double axialFraction = 0;
            double lateralFraction = 0;
            if (totalDistance > 0.001) {
                axialFraction = axialDistance / totalDistance;
                lateralFraction = lateralDistance / totalDistance;
            }

            // Use these fractions to drive at the correct direction from the start.
            driveFieldCentric(axialFraction * maxSpeed, lateralFraction * maxSpeed, 0);

            /*
            Keep looping while we are still active, and there is time left, and all motors are running. Note, We use
            (isBusy() && isBusy() && isBusy()) in the loop test, which means that when ANY motor hits its target
            position, the motion will stop. This is "safer" in the event that the robot will always end the motion as
            soon as possible. However, if you require that ALL motors have finished their moves before the robot
            continues onto the next step, use (isBusy() || isBusy() || isBusy() || isBusy()) in the loop test.
             */
            while (myOpMode.opModeIsActive()) {

                // Calculate how far each wheel is from its target
                double leftFrontError = Math.abs(leftFrontTarget - leftFrontDrive.getCurrentPosition());
                double leftBackError = Math.abs(leftBackTarget - leftBackDrive.getCurrentPosition());
                double rightFrontError = Math.abs(rightFrontTarget - rightFrontDrive.getCurrentPosition());
                double rightBackError = Math.abs(rightBackTarget - rightBackDrive.getCurrentPosition());

                // Find the max distance among the four wheels
                double maxError =
                        Math.max(Math.max(leftFrontError, leftBackError),
                                Math.max(rightFrontError, rightBackError));

                // Determine required steering to keep on heading
                yawPower = getSteeringCorrection(desiredHeading, P_AXIAL_GAIN);

                // If the greatest distance remaining is within out tolerance we are done
                if (maxError <= TOLERANCE_TICKS) {
                    break;
                }

                // if driving in reverse, the motor correction also needs to be reversed
                if (axialDistance < 0)
                    yawPower *= -1.0;

                // Apply the turning correction to the current driving power.
                driveFieldCentric(axialFraction * maxSpeed, lateralFraction * maxSpeed, yawPower);

                // Display drive status for the driver.
                sendTelemetry(true);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            driveFieldCentric(0, 0, 0);
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Spin on the central axis to point in a new direction. Move will stop if either of these conditions occur:
     * 1) Move gets to the heading (angle), or 2) Driver stops the OpMode running.
     *
     * @param maxYawSpeed   Desired MAX speed of turn. (range 0 to +1.0)
     * @param heading       Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                      0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                      If a relative angle is required, add/subtract from current heading.
     *
     */
    public void turnToHeading(double maxYawSpeed, double heading) {
        // Run getSteeringCorrection() once to  pre-calculate the current error
        getSteeringCorrection(heading, P_AXIAL_GAIN);

        // Keep looping while we are still active, and not on heading.
        while (myOpMode.opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

            // Determine required steering to keep on heading
            yawPower = getSteeringCorrection(heading, P_YAW_GAIN);

            // Clip the speed to the maximum permitted value.
            yawPower = Range.clip(yawPower, -maxYawSpeed, maxYawSpeed);

            // Pivot in place by applying the turning correction
            driveFieldCentric(0, 0, yawPower);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion
        driveFieldCentric(0, 0, 0);
    }

    /**
     *  Obtain & hold a heading for a finite amount of time. Move will stop once the requested time has elapsed.
     *  This function is useful for giving the robot a moment to stabilize its heading between movements.
     *
     * @param maxYawSpeed   Maximum differential turn speed (range 0 to +1.0)
     * @param heading       Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                      0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                      If a relative angle is required, add/subtract from current heading.
     * @param holdTime      Length of time (in seconds) to hold the specified heading.
     */
    public void holdHeading(double maxYawSpeed, double heading, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        // keep looping while we have time remaining.
        while (myOpMode.opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Determine required steering to keep on heading
            yawPower = getSteeringCorrection(heading, P_YAW_GAIN);

            // Clip the speed to the maximum permitted value.
            yawPower = Range.clip(yawPower, -maxYawSpeed, maxYawSpeed);

            // Pivot in place by applying the turning correction
            driveFieldCentric(0, 0, yawPower);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        driveFieldCentric(0, 0, 0);
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
        headingError = -targetHeading + getHeading();

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
     * @param axial     Forward/Reverse driving power (-1.0 to 1.0) +ve is forward
     * @param lateral    Right/Left driving power (-1.0 to 1.0) +ve is right
     * @param yaw      Right/Left turning power (-1.0 to 1.0) +ve is clockwise
     */
    public void driveRobotCentric(double axial, double lateral, double yaw) {

        axialPower = axial;
        lateralPower = lateral;
        yawPower = yaw;

        double max;

        /*
         Combine the joystick requests for each axis-motion to determine each wheel's power. Set up a variable for each
         drive wheel to save the power level for telemetry. Denominator is the largest motor power (absolute value) or
         1. This ensures all the powers maintain the same ratio, but only when at least one is out of the range [-1, 1]
         */
        double denominator = Math.max(Math.abs(axial) + Math.abs(lateral) + Math.abs(yaw), 1);
        leftFrontPower = (axial + lateral + yaw) / denominator;
        leftBackPower = (axial - lateral + yaw) / denominator;
        rightFrontPower = (axial - lateral - yaw) / denominator;
        rightBackPower = (axial + lateral - yaw) / denominator;

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
     * Calculate the motor powers required to achieve the requested robot motions:
     * Drive (Axial motion), Strafe (Lateral motion), and Turn (Yaw motion)
     * Then send these power levels to the motors.
     *
     * @param axial     Forward/Reverse driving power (-1.0 to 1.0) +ve is forward
     * @param lateral    Right/Left driving power (-1.0 to 1.0) +ve is right
     * @param yaw      Right/Left turning power (-1.0 to 1.0) +ve is clockwise
     */
    public void driveFieldCentric(double axial, double lateral, double yaw) {

        axialPower = axial;
        lateralPower = lateral;
        yawPower = yaw;

        double max;
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot rotation
        double lateralRotation = lateral * Math.cos(-botHeading) - axial * Math.sin(-botHeading);
        double axialRotation = lateral * Math.sin(-botHeading) + axial * Math.cos(-botHeading);

        /*
         Combine the joystick requests for each axis-motion to determine each wheel's power. Set up a variable for each
         drive wheel to save the power level for telemetry. Denominator is the largest motor power (absolute value) or
         1. This ensures all the powers maintain the same ratio, but only when at least one is out of the range [-1, 1]
         */
        double denominator = Math.max(Math.abs(axialRotation) + Math.abs(lateralRotation) + Math.abs(yaw), 1);
        leftFrontPower = (axialRotation + lateralRotation + yaw) / denominator;
        leftBackPower = (axialRotation - lateralRotation + yaw) / denominator;
        rightFrontPower = (axialRotation - lateralRotation - yaw) / denominator;
        rightBackPower = (axialRotation + lateralRotation - yaw) / denominator;

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

    /**
     *  Display the various control parameters while driving
     *
     * @param axial     Set to true if we are driving straight, and the encoder positions should be included
     *                  in the telemetry.
     */
    private void sendTelemetry(boolean axial) {

        if (axial) {
            myOpMode.telemetry.addData("Motion", "Drive Axial");
            myOpMode.telemetry.addData("Target Pos LF:LB:RF:RB", "%7d:%7d:%7d:%7d",
                    leftFrontTarget, leftBackTarget, rightFrontTarget, rightBackTarget);
            myOpMode.telemetry.addData("Actual Pos LF:LB:RF:RB", "%7d:%7d:%7d:%7d",
                    leftFrontDrive.getCurrentPosition(), leftBackDrive.getCurrentPosition(),
                    rightFrontDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
        } else {
            myOpMode.telemetry.addData("Motion", "Turning");
        }

        myOpMode.telemetry.addData("Heading- Target : Current", "%5.2f : %5.0f", targetHeading, getHeading());
        myOpMode.telemetry.addData("Error  : Steer Pwr",  "%5.1f : %5.1f", headingError, yawPower);
        myOpMode.telemetry.addData("Wheel Speeds L : R", "%5.2f : %5.2f : %5.2f : %5.2f",
                leftFrontPower, leftBackPower, rightFrontPower, rightBackPower);
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
    public void setVerticalArmPosition(double position) {
        // leftVerticalArmDrive.setTargetPosition((int) (position));
        rightVerticalArmDrive.setTargetPosition((int) (position));

        // ((DcMotorEx) leftVerticalArmDrive).setVelocity(2100);
        ((DcMotorEx) rightVerticalArmDrive).setVelocity(2100);
        // leftVerticalArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightVerticalArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Send the two horizontalArm-servos to opposing (mirrored) positions.
     *
     * @param position  (0.0 to 1.0) +ve is extended
     */
    public void setHorizontalArmPosition(double position) {
        // limit servo range to the range of the horizontal arm.
        position /= 20;

        leftHorizontalArmDrive.setPosition(position + 0.005);
        rightHorizontalArmDrive.setPosition(1.0 - (position + 0.002));
    }

    /**
     * Send the verticalWrist-servo to position
     *
     * @param position  (0.0 to 1.0) +ve is extended
     */
    public void setVerticalWristPosition(double position) {
        verticalWristDrive.setPosition(position);
    }

    /**
     * Send the horizontalWrist-servo to position
     *
     * @param position  (0.0 to 1.0) +ve is extended
     */
    public void setHorizontalWristPosition(double position) {
        horizontalWristDrive.setPosition(position);
    }

    /**
     * Send the claw-servo to position
     *
     * @param position  (0.0 to 1.0) +ve is open
     */
    public void setVerticalClawPosition(double position) {
        verticalClawDrive.setPosition(position);
    }

    /**
     * Send the claw-servo to position
     *
     * @param position  (0.0 to 1.0) +ve is open
     */
    public void setHorizontalClawPosition(double position) {
        horizontalClawDrive.setPosition(position);
    }
}