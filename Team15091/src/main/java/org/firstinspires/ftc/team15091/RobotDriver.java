package org.firstinspires.ftc.team15091;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class RobotDriver {
    private Robot _robot;

    private double headingError = 0;

    // These variable are declared here (as class members) so they can be updated in various methods,
    // but still be displayed by sendTelemetry()
    private double targetHeading = 0;
    private double driveSpeed = 0;
    private double turnSpeed = 0;
    private double sideSpeed = 0;
    private double leftFrontSpeed = 0;
    private double leftRearSpeed = 0;
    private double rightFrontSpeed = 0;
    private double rightRearSpeed = 0;
    private int leftFrontTarget = 0;
    private int leftRearTarget = 0;
    private int rightFrontTarget = 0;
    private int rightRearTarget = 0;

    private final ElapsedTime _runtime = new ElapsedTime();
    private LinearOpMode _opMode;
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double P_TURN_GAIN = 0.02;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_GAIN = 0.03;     // Larger is more responsive, but also less stable
    private static final double HEADING_THRESHOLD = 0.36d;      // As tight as we can make it with an integer gyro

    RobotDriver(Robot robot, LinearOpMode opMode) {
        _robot = robot;
        _opMode = opMode;

        _robot.resetDrive();
    }

    public final boolean gyroDrive(double maxDriveSpeed,
                                   double distance,
                                   double heading,
                                   double timeoutS,
                                   IObjectDetector<Boolean> objectDetector) {
        boolean successful = true;
        // Ensure that the opmode is still active
        if (_opMode.opModeIsActive()) {
            // Set Target and Turn On RUN_TO_POSITION
            _robot.setDriveTarget(distance, false);
            _robot.setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            _runtime.reset();

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);

            moveRobot(maxDriveSpeed, 0, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (_opMode.opModeIsActive() &&
                    _runtime.seconds() < timeoutS &&
                    _robot.isDriveBusy()) {

                _opMode.idle();

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN, maxDriveSpeed);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                if (objectDetector != null && objectDetector.objectDetected()) {
                    _robot.beep();
                    successful = false;
                    break;
                }

                // Apply the turning correction to the current driving speed.
                moveRobot(driveSpeed, turnSpeed, 0);

                // Display drive status for the driver.
                sendTelemetry(true);
            }

            // Stop all motion;
            moveRobot(0, 0, 0);
            _robot.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        return successful;
    }

    public final void gyroSlide(double maxSlideSpeed,
                                double distance,
                                double heading,
                                double timeoutS,
                                IObjectDetector<Boolean> objectDetector) {

        // Ensure that the opmode is still active
        if (_opMode.opModeIsActive()) {
            // Set Target and Turn On RUN_TO_POSITION
            _robot.setDriveTarget(distance, true);
            _robot.setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            _runtime.reset();

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxSlideSpeed = Math.abs(maxSlideSpeed);
            moveRobot(0, 0, maxSlideSpeed);

            // keep looping while we are still active, and BOTH motors are running.
            while (_opMode.opModeIsActive() &&
                    _runtime.seconds() < timeoutS &&
                    _robot.isDriveBusy()) {
                _opMode.idle();

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN, maxSlideSpeed);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                if (objectDetector != null && objectDetector.objectDetected()) {
                    _robot.beep();
                    break;
                }

                // Apply the turning correction to the current driving speed.
                moveRobot(0, 0, sideSpeed);

                // Display drive status for the driver.
                sendTelemetry(true);
            }

            // Stop all motion;
            moveRobot(0, 0, 0);
            _robot.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public final void gyroTurn(double maxTurnSpeed, double heading, double timeoutS) {
        _runtime.reset();

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN, maxTurnSpeed);

        // keep looping while we are still active, and not on heading.
        while (_opMode.opModeIsActive() &&
                _runtime.seconds() < timeoutS &&
                (Math.abs(headingError) > HEADING_THRESHOLD)) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN, 1d);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed, 0);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0, 0);
    }

    /**
     * Obtain & hold a heading for a finite amount of time
     * <p>
     * Move will stop once the requested time has elapsed
     * <p>
     * This function is useful for giving the robot a moment to stabilize it's heading between movements.
     *
     * @param maxTurnSpeed Maximum differential turn speed (range 0 to +1.0)
     * @param heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                     0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                     If a relative angle is required, add/subtract from current heading.
     * @param holdTime     Length of time (in seconds) to hold the specified heading.
     */
    public final void gyroHold(double maxTurnSpeed, double heading, double holdTime) {
        _runtime.reset();

        // keep looping while we have time remaining.
        while (_opMode.opModeIsActive() && (_runtime.seconds() < holdTime)) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN, maxTurnSpeed);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed, 0);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0, 0);
    }

    // **********  LOW Level driving functions.  ********************

    /**
     * Use a Proportional Controller to determine how much steering correction is required.
     *
     * @param desiredHeading   The desired absolute heading (relative to last heading reset)
     * @param proportionalGain Gain factor applied to heading error to obtain turning power.
     * @param maxSpeed         Maximum drive speed to use.
     * @return Turning power needed to get to required heading.
     */
    public double getSteeringCorrection(double desiredHeading, double proportionalGain, double maxSpeed) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Determine the heading current error
        headingError = targetHeading - _robot.getHeading();

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -Math.abs(maxSpeed), Math.abs(maxSpeed));
    }

    /**
     * Take separate drive (fwd/rev) and turn (right/left) requests,
     * combines them, and applies the appropriate speed commands to the left and right wheel motors.
     *
     * @param drive forward motor speed
     * @param turn  clockwise turning motor speed.
     */
    public void moveRobot(double drive, double turn, double side) {
        driveSpeed = drive;    // save this value as a class member so it can be used by telemetry.
        turnSpeed = turn;      // save this value as a class member so it can be used by telemetry.
        sideSpeed = side;      // save this value as a class member so it can be used by telemetry.

        leftFrontSpeed = drive - turn + side;
        leftRearSpeed = drive - turn - side;
        rightFrontSpeed = drive + turn - side;
        rightRearSpeed = drive + turn + side;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(
                Math.max(Math.abs(leftFrontSpeed), Math.abs(leftRearSpeed)),
                Math.max(Math.abs(rightFrontSpeed), Math.abs(rightRearSpeed)));
        if (max > 1.0) {
            leftFrontSpeed /= max;
            leftRearSpeed /= max;
            rightFrontSpeed /= max;
            rightRearSpeed /= max;
        }

        _robot.setDrivePower(leftFrontSpeed, leftRearSpeed, rightFrontSpeed, rightRearSpeed);
    }

    /**
     * Display the various control parameters while driving
     *
     * @param straight Set to true if we are driving straight, and the encoder positions should be included in the telemetry.
     */
    private void sendTelemetry(boolean straight) {

        if (straight) {
            _opMode.telemetry.addData("Motion", "Drive Straight");
            _opMode.telemetry.addData("Target Pos L:R", "%7d:%7d:%7d:%7d",
                    leftFrontTarget, leftRearTarget,
                    rightFrontTarget, rightRearTarget);
            _opMode.telemetry.addData("Actual Pos L:R", "%7d:%7d:%7d:%7d",
                    _robot.leftFront.getCurrentPosition(),
                    _robot.rightFront.getCurrentPosition(),
                    _robot.leftRear.getCurrentPosition(),
                    _robot.rightRear.getCurrentPosition());
        } else {
            _opMode.telemetry.addData("Motion", "Turning");
        }

        _opMode.telemetry.addData("Heading- Target : Current", "%5.2f : %5.0f", targetHeading, _robot.getHeading());
        _opMode.telemetry.addData("Error  : Steer Pwr", "%5.1f : %5.1f", headingError, turnSpeed);
        _opMode.telemetry.addData("Wheel Speeds L : R", "%5.2f : %5.2f : %5.2f : %5.2f",
                leftFrontSpeed, leftRearSpeed, rightFrontSpeed, rightRearSpeed);
        _opMode.telemetry.update();
    }
}
