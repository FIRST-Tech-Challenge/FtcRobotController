package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class DrivingFunctions {
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private IMU imu = null;
    private LinearOpMode lom = null;

    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;   // Our motors are these: https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 3.78 ;     // These are our Mecanum wheels (96mm) - https://www.gobilda.com/96mm-mecanum-wheel-set-70a-durometer-bearing-supported-rollers/
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.4;     // Max driving speed for better distance accuracy.
    static final double     TURN_SPEED              = 0.2;     // Max Turn speed to limit turn rate
    static final double     HEADING_THRESHOLD       = 1.0 ;    // How close must the heading get to the target before moving to next step.
    static final double     P_TURN_GAIN            = 0.02;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_GAIN           = 0.03;     // Larger is more responsive, but also less stable

    // These variable are declared here (as class members) so they can be updated in various methods,
    // but still be displayed by sendTelemetry()
    private double  targetHeading = 0;
    private double  driveSpeed    = 0;
    private double  turnSpeed     = 0;
    private double  leftFrontSpeed     = 0;
    private double  leftBackSpeed     = 0;
    private double  rightFrontSpeed     = 0;
    private double  rightBackSpeed    = 0;
    private int     leftFrontTarget    = 0;
    private int     rightFrontTarget   = 0;
    private int     leftBackTarget    = 0;
    private int     rightBackTarget   = 0;
    private double  headingError  = 0;

    public DrivingFunctions(LinearOpMode l)
    {
        lom = l;
        Initialize();
    }
    private void Initialize()
    {
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = lom.hardwareMap.get(DcMotor.class, "frontleft");
        leftBackDrive  = lom.hardwareMap.get(DcMotor.class, "backleft");
        rightFrontDrive = lom.hardwareMap.get(DcMotor.class, "frontright");
        rightBackDrive = lom.hardwareMap.get(DcMotor.class, "backright");

        imu = lom.hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        imu.resetYaw();

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Ensure the robot is stationary.  Reset the encoders and set the motors to BRAKE mode
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set the encoders for closed loop speed control
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        imu.resetYaw();
    }


    /**
     *  Drive in a straight line, on a fixed compass heading (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the OpMode running.
     *
     * @param maxDriveSpeed MAX Speed for forward/rev motion (range 0 to +1.0) .
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backward.
     * @param heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from the current robotHeading.
     */
    public void driveStraight(double maxDriveSpeed,
                              double distance,
                              double heading) {

        // Ensure that the OpMode is still active
        if (lom.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int)(distance * COUNTS_PER_INCH);
            leftFrontTarget = leftFrontDrive.getCurrentPosition() + moveCounts;
            rightFrontTarget = rightFrontDrive.getCurrentPosition() + moveCounts;
            leftBackTarget = leftBackDrive.getCurrentPosition() + moveCounts;
            rightBackTarget = rightBackDrive.getCurrentPosition() + moveCounts;

            // Set Target FIRST, then turn on RUN_TO_POSITION
            leftFrontDrive.setTargetPosition(leftFrontTarget);
            rightFrontDrive.setTargetPosition(rightFrontTarget);
            leftBackDrive.setTargetPosition(leftBackTarget);
            rightBackDrive.setTargetPosition(rightBackTarget);

            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (lom.opModeIsActive() &&
                    (leftFrontDrive.isBusy() && rightFrontDrive.isBusy() && leftBackDrive.isBusy() && rightBackDrive.isBusy())) {
                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                //moveRobot(driveSpeed, turnSpeed);

                // Display drive status for the driver.
                sendTelemetry(true);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0);
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     *  Spin on the central axis to point in a new direction.
     *  <p>
     *  Move will stop if either of these conditions occur:
     *  <p>
     *  1) Move gets to the heading (angle)
     *  <p>
     *  2) Driver stops the OpMode running.
     *
     * @param maxTurnSpeed Desired MAX speed of turn. (range 0 to +1.0)
     * @param heading Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void turnToHeading(double maxTurnSpeed, double heading) {

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        while (lom.opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }

    /**
     *  Obtain & hold a heading for a finite amount of time
     *  <p>
     *  Move will stop once the requested time has elapsed
     *  <p>
     *  This function is useful for giving the robot a moment to stabilize it's heading between movements.
     *
     * @param maxTurnSpeed      Maximum differential turn speed (range 0 to +1.0)
     * @param heading    Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        // keep looping while we have time remaining.
        while (lom.opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }

    // **********  LOW Level driving functions.  ********************

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
     * Take separate drive (fwd/rev) and turn (right/left) requests,
     * combines them, and applies the appropriate speed commands to the left and right wheel motors.
     * @param drive forward motor speed
     * @param turn  clockwise turning motor speed.
     */
    public void moveRobot(double drive, double turn) {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.

        leftFrontSpeed  = drive - turn;
        leftBackSpeed = drive - turn;
        rightFrontSpeed  = drive + turn;
        rightBackSpeed = drive + turn;

        double max = Math.max(Math.abs(leftFrontSpeed), Math.abs(rightFrontSpeed));
        max = Math.max(max, Math.abs(leftBackSpeed));
        max = Math.max(max, Math.abs(rightBackSpeed));

        if (max > 1.0) {
            leftFrontSpeed /= max;
            rightFrontSpeed /= max;
            leftBackSpeed /= max;
            rightBackSpeed /= max;
        }

        leftFrontDrive.setPower(leftFrontSpeed);
        rightFrontDrive.setPower(rightFrontSpeed);
        leftBackDrive.setPower(leftBackSpeed);
        rightBackDrive.setPower(rightBackSpeed);
    }

    /**
     *  Display the various control parameters while driving
     *
     * @param straight  Set to true if we are driving straight, and the encoder positions should be included in the telemetry.
     */
    private void sendTelemetry(boolean straight) {

        if (straight) {
            lom.telemetry.addData("Motion", "Drive Straight");
            lom.telemetry.addData("Target Pos L:R",  "%7d:%7d",      leftFrontTarget,  rightFrontTarget);
            lom.telemetry.addData("Actual Pos L:R",  "%7d:%7d",      leftFrontDrive.getCurrentPosition(),
                    rightFrontDrive.getCurrentPosition());
        } else {
            lom.telemetry.addData("Motion", "Turning");
        }

        lom.telemetry.addData("Heading- Target : Current", "%5.2f : %5.0f", targetHeading, getHeading());
        lom.telemetry.addData("Error  : Steer Pwr",  "%5.1f : %5.1f", headingError, turnSpeed);
        lom.telemetry.addData("Wheel Speeds L : R", "%5.2f : %5.2f", leftFrontSpeed, rightFrontSpeed);
        lom.telemetry.update();
    }

    /**
     * read the Robot heading directly from the IMU (in degrees)
     */
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }








    // ************************************************************
    // Preston's driving functions
    // ************************************************************
    public void stopM() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
    public void waitStopped(int time){
        stopM();
        lom.sleep(time);
    }
    public void rotateFieldCentric(double power, int degrees){
        double TargetAngle = degrees;
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double deltaDegrees = (TargetAngle - botHeading + 540) % 360 - 180;
        double yaw = 0;
        double kp = -0.033;
        while (lom.opModeIsActive()){

            if ((Math.abs(deltaDegrees) < 1.0 && Math.abs(power) < 2.0)){
                break;
            }
            else {
                yaw = kp * deltaDegrees / power;
            }
            double leftFrontPower  = yaw;
            double rightFrontPower = -yaw;
            double leftBackPower   = yaw;
            double rightBackPower  = -yaw;
            leftFrontDrive.setPower(leftFrontPower * power);
            rightFrontDrive.setPower(rightFrontPower * power);
            leftBackDrive.setPower(leftBackPower * power);
            rightBackDrive.setPower(rightBackPower * power);

        }
    }
    public void rotateDegrees(double power, int degrees){
        double TargetAngle = degrees;
        int stop = 1000;
        while (stop > 0){
            stop -= 1;
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double deltaDegrees = (TargetAngle - botHeading + 540) % 360 - 180;
            double yaw = 0;
            double kp = -0.033;
            if ((Math.abs(deltaDegrees) < 1.0 && Math.abs(power) < 2.0)){
                break;
            }
            else {
                yaw = kp * deltaDegrees / power;
            }
            double leftFrontPower  = yaw;
            double rightFrontPower = -yaw;
            double leftBackPower   = yaw;
            double rightBackPower  = -yaw;
            leftFrontDrive.setPower(leftFrontPower * power);
            rightFrontDrive.setPower(rightFrontPower * power);
            leftBackDrive.setPower(leftBackPower * power);
            rightBackDrive.setPower(rightBackPower * power);

        }
    }
    public void driveForward(double power, int milliseconds) {
        int direction = 1;
        leftFrontDrive.setPower(power * direction);
        rightFrontDrive.setPower(power * direction);
        leftBackDrive.setPower(power * direction);
        rightBackDrive.setPower(power * direction);
        lom.sleep(milliseconds);
        stopM();
    }


    public void driveBackward(double power, int milliseconds) {
        int direction = -1;
        leftFrontDrive.setPower(power * direction);
        rightFrontDrive.setPower(power * direction);
        leftBackDrive.setPower(power * direction);
        rightBackDrive.setPower(power * direction);
        lom.sleep(milliseconds);
        stopM();
    }

    public void rotateRight(double power, int milliseconds) {
        int direction = 1;
        leftFrontDrive.setPower(power * direction);
        rightFrontDrive.setPower(power * direction * -1);
        leftBackDrive.setPower(power * direction);
        rightBackDrive.setPower(power * direction * -1);
        lom.sleep(milliseconds);
        stopM();
    }

    public void rotateLeft(double power, int milliseconds) {
        int direction = -1;
        leftFrontDrive.setPower(power * direction);
        rightFrontDrive.setPower(power * direction * -1);
        leftBackDrive.setPower(power * direction);
        rightBackDrive.setPower(power * direction * -1);
        lom.sleep(milliseconds);
        stopM();
    }

    public void strafeRight(double power, int milliseconds) {
        int direction = 1;
        leftFrontDrive.setPower(power * direction * -1);
        rightFrontDrive.setPower(power * direction);
        leftBackDrive.setPower(power * direction * -1);
        rightBackDrive.setPower(power * direction);
        lom.sleep(milliseconds);
        stopM();
    }

    public void strafeLeft(double power, int milliseconds) {
        int direction = -1;
        leftFrontDrive.setPower(power * direction * -1);
        rightFrontDrive.setPower(power * direction);
        leftBackDrive.setPower(power * direction * -1);
        rightBackDrive.setPower(power * direction);
        lom.sleep(milliseconds);
        stopM();
    }

}
