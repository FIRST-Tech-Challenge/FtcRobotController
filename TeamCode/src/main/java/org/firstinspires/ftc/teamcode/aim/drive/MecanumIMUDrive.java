package org.firstinspires.ftc.teamcode.aim.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class MecanumIMUDrive {
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private IMU imu = null;
    private LinearOpMode curOpMode;
    private double turnGain = 0.02;
    private double driveGain = 0.03;
    private double headingThreshold = 1.0;
    private double countsPerInch;
    private boolean debug;

    private double moveTargetHeading = 0;
    private boolean stopped = true;

    public class InitParams {
        public LinearOpMode opMode;
        public String imuName;
        public String frontLeftWheelName;
        public String frontRightWheelName;
        public String backLeftWheelName;
        public String backRightWheelName;
        public double countPerMotorRev;
        public double driveGearReduction;
        public double wheelDiameterInches;
        public boolean debug;
    }

    public InitParams defaultParams() {
        return new InitParams();
    }

    private void initDriveMotor(DcMotor motor) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void init(InitParams params) {
        curOpMode = params.opMode;
        frontLeftDrive = curOpMode.hardwareMap.get(DcMotor.class, params.frontLeftWheelName);
        frontRightDrive = curOpMode.hardwareMap.get(DcMotor.class, params.frontRightWheelName);
        backLeftDrive = curOpMode.hardwareMap.get(DcMotor.class, params.backLeftWheelName);
        backRightDrive = curOpMode.hardwareMap.get(DcMotor.class, params.backRightWheelName);
        countsPerInch = (params.countPerMotorRev * params.driveGearReduction) /
                (params.wheelDiameterInches * 3.1415);
        imu = curOpMode.hardwareMap.get(IMU.class, params.imuName);
        debug = params.debug;

        initDriveMotor(frontLeftDrive);
        initDriveMotor(frontRightDrive);
        initDriveMotor(backLeftDrive);
        initDriveMotor(backRightDrive);

        this.moveTargetHeading = this.getHeading();
    }

    /**
     * Drive in the axial (forward/reverse) direction, maintain the current heading and don't drift sideways
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the desired position
     * 2) Driver stops the OpMode running.
     *
     * @param maxDriveSpeed MAX Speed for forward/rev motion (range 0 to +1.0) .
     * @param minDriveSpeed MIN Speed for forward/rev motion (range 0 to +1.0) .
     *                      When the position is close to the target position, MIN speed is used.
     * @param distance      Distance (in inches) to move from current position.  Negative distance means move backward.
     */
    public void drive(double maxDriveSpeed, double minDriveSpeed, double distance) {
        driveStraight(maxDriveSpeed, minDriveSpeed, distance, false);
    }

    /**
     * Strafe in the lateral (left/right) direction, maintain the current heading and don't drift fwd/bwd
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the desired position
     * 2) Driver stops the OpMode running.
     *
     * @param maxDriveSpeed MAX Speed for forward/rev motion (range 0 to +1.0) .
     * @param minDriveSpeed MIN Speed for forward/rev motion (range 0 to +1.0) .
     *                      When the position is close to the target position, MIN speed is used.
     * @param distance      Distance (in inches) to move from current position.  Negative distance means move backward.
     */
    public void strafe(double maxDriveSpeed, double minDriveSpeed, double distance) {
        driveStraight(maxDriveSpeed, minDriveSpeed, distance, true);
    }

    /**
     * Drive in a straight line, on a fixed compass heading (angle), based on encoder counts.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the desired position
     * 2) Driver stops the OpMode running.
     *
     * @param maxDriveSpeed MAX Speed for forward/rev motion (range 0 to +1.0) .
     * @param minDriveSpeed MIN Speed for forward/rev motion (range 0 to +1.0) .
     *                      When the position is close to the target position, MIN speed is used.
     * @param distance      Distance (in inches) to move from current position.  Negative distance means move backward.
     * @param strafe        Move in the axial or lateral direction.
     */
    private void driveStraight(double maxDriveSpeed, double minDriveSpeed,
                               double distance, boolean strafe) {


        // Ensure that the OpMode is still active
        if (curOpMode.opModeIsActive()) {
            double heading = getHeading();
            if (this.debug) {
                curOpMode.telemetry.addData("heading", "%f", heading);
            }
            int frontLeftTarget, backLeftTarget, frontRightTarget, backRightTarget;
            // Determine new target position, and pass to motor controller
            int moveCounts = (int) (distance * countsPerInch);
            int origPosition = frontLeftDrive.getCurrentPosition();

            if (!strafe) {
                frontLeftTarget = frontLeftDrive.getCurrentPosition() + moveCounts;
                backLeftTarget = backLeftDrive.getCurrentPosition() + moveCounts;
                frontRightTarget = frontRightDrive.getCurrentPosition() + moveCounts;
                backRightTarget = backRightDrive.getCurrentPosition() + moveCounts;
            } else {
                frontLeftTarget = frontLeftDrive.getCurrentPosition() + moveCounts;
                backLeftTarget = backLeftDrive.getCurrentPosition() - moveCounts;
                frontRightTarget = frontRightDrive.getCurrentPosition() - moveCounts;
                backRightTarget = backRightDrive.getCurrentPosition() + moveCounts;
            }

            if (debug) {
                curOpMode.telemetry.addData("Target Pos FL:FR", "%7d:%7d",
                        frontLeftTarget, frontRightTarget);
                curOpMode.telemetry.addData("Target Pos BL:BR", "%7d:%7d",
                        backLeftTarget, backRightTarget);
            }

            // Set Target FIRST, then turn on RUN_TO_POSITION
            frontLeftDrive.setTargetPosition(frontLeftTarget);
            backLeftDrive.setTargetPosition(backLeftTarget);
            frontRightDrive.setTargetPosition(frontRightTarget);
            backRightDrive.setTargetPosition(backRightTarget);

            frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            minDriveSpeed = Math.abs(minDriveSpeed);

            double driveSpeed = this.getDriveCorrection(moveCounts,
                    frontLeftDrive.getCurrentPosition() - origPosition,
                    maxDriveSpeed, minDriveSpeed,
                    this.driveGain);
            moveRobot(strafe ? 0 : driveSpeed, 0, strafe ? driveSpeed : 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (curOpMode.opModeIsActive() &&
                    (frontLeftDrive.isBusy() && frontRightDrive.isBusy() &&
                            backLeftDrive.isBusy()) && backRightDrive.isBusy()) {

                // Determine required steering to keep on heading
                double turnSpeed = getSteeringCorrection(heading, driveGain);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                driveSpeed = this.getDriveCorrection(moveCounts,
                        frontLeftDrive.getCurrentPosition() - origPosition,
                        maxDriveSpeed, minDriveSpeed,
                        this.driveGain);
                if (this.debug) {
                    this.curOpMode.telemetry.addData("speed", "%f", driveSpeed);
                }
                // Apply the turning correction to the current driving speed.
                moveRobot(strafe ? 0 : driveSpeed, turnSpeed, strafe ? driveSpeed : 0);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0, 0);
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Spin on the central axis to point in a new direction.
     * <p>
     * Move will stop if either of these conditions occur:
     * <p>
     * 1) Move gets to the heading (angle)
     * <p>
     * 2) Driver stops the OpMode running.
     *
     * @param maxTurnSpeed Desired MAX speed of turn. (range 0 to +1.0)
     * @param heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                     0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                     If a relative angle is required, add/subtract from current heading.
     */
    public void turnToHeading(double maxTurnSpeed, double heading) {

        // keep looping while we are still active, and not on heading.
        while (curOpMode.opModeIsActive() && (Math.abs(getSteeringError(heading)) > headingThreshold)) {

            // Determine required steering to keep on heading
            double turnSpeed = getSteeringCorrection(heading, turnGain);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed, 0);
        }

        // Stop all motion;
        moveRobot(0, 0, 0);
    }

    /**
     * Obtain & hold a heading for a finite amount of time
     * <p>
     * Move will stop once the requested time has elapsed
     * <p>
     * This function is useful for giving the robot a moment to stabilize its heading between movements.
     *
     * @param maxTurnSpeed Maximum differential turn speed (range 0 to +1.0)
     * @param heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                     0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                     If a relative angle is required, add/subtract from current heading.
     * @param holdTime     Length of time (in seconds) to hold the specified heading.
     */
    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        // keep looping while we have time remaining.
        while (curOpMode.opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Determine required steering to keep on heading
            double turnSpeed = getSteeringCorrection(heading, turnGain);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed, 0);
        }

        // Stop all motion;
        moveRobot(0, 0, 0);
    }

    /**
     * Use a Proportional Controller to determine how much steering correction is required.
     *
     * @param desiredHeading   The desired absolute heading (relative to last heading reset)
     * @param proportionalGain Gain factor applied to heading error to obtain turning power.
     * @return Turning power needed to get to required heading.
     */
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        // Determine the heading current error
        double headingError = getSteeringError(desiredHeading);
        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    public double getSteeringError(double desiredHeading) {
        double targetHeading = desiredHeading;  // Save for telemetry

        // Determine the heading current error
        double headingError = targetHeading - getHeading();

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        return headingError;
    }

    public double getDriveCorrection(double targetPosition, double curPosition, double maxPower, double minPower, double proportionalGain) {
        double err =  Math.abs(targetPosition - curPosition) / countsPerInch;
        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        double power = Range.clip(err * proportionalGain, -maxPower, maxPower);
        return Math.max(power, minPower);
    }

    /**
     * Take separate drive (fwd/rev) and turn (right/left) requests,
     * combines them, and applies the appropriate speed commands to the left and right wheel motors.
     *
     * @param drive forward motor speed
     * @param turn  clockwise turning motor speed.
     */
    public void moveRobot(double drive, double turn, double strafe) {
        double frontLeftSpeed = drive - turn + strafe;
        double frontRightSpeed = drive + turn - strafe;
        double backLeftSpeed = drive - turn - strafe;
        double backRightSpeed = drive + turn + strafe;

        double max0 = Math.max(Math.abs(frontLeftSpeed), Math.abs(frontRightSpeed));
        double max1 = Math.max(Math.abs(backLeftSpeed), Math.abs(backRightSpeed));
        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(max0, max1);
        if (max > 1.0) {
            frontLeftSpeed /= max;
            frontRightSpeed /= max;
            backLeftSpeed /= max;
            backRightSpeed /= max;
        }

        if (debug) {
            curOpMode.telemetry.addData("Move Robot", "%f:%f:%f;%f",
                    frontLeftSpeed, frontRightSpeed, backLeftSpeed, backRightSpeed);
        }

        frontLeftDrive.setPower(frontLeftSpeed);
        frontRightDrive.setPower(frontRightSpeed);
        backLeftDrive.setPower(backLeftSpeed);
        backRightDrive.setPower(backRightSpeed);
    }

    public void moveRobotX(double power, double x,  double y, double turn) {
        double theta = Math.atan2(y, x);

        double sin = Math.sin(theta - Math.PI/4);
        double cos = Math.cos(theta - Math.PI/4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        double leftFrontPower = power * cos/max + turn;
        double rightFrontPower = power * sin/max - turn;
        double leftBackPower = power * sin/max + turn;
        double rightBackPower = power * cos/max - turn;

        if ((power + Math.abs(turn)) > 1) {
            leftFrontPower /= power + Math.abs(turn);
            rightFrontPower /= power + Math.abs(turn);
            leftBackPower /= power + Math.abs(turn);
            rightBackPower /= power + Math.abs(turn);
        }

        this.frontLeftDrive.setPower(leftFrontPower);
        this.frontRightDrive.setPower(rightFrontPower);
        this.backLeftDrive.setPower(leftBackPower);
        this.backRightDrive.setPower(rightBackPower);
    }

    public void moveByPower(double power, double x,  double y,  double turn) {
        double maxDriveSpeed = Math.abs(power);
        double turnSpeed = 0;

        if (x != 0 || y != 0 || turn != 0) {
            if (this.stopped) {
                this.moveTargetHeading = getHeading();
            }
            this.moveTargetHeading += turn;
            this.stopped = false;
            if (Math.abs(this.moveTargetHeading - getHeading()) > 1) {
                turnSpeed = -getSteeringCorrection(this.moveTargetHeading, driveGain);
            }
            moveRobotX(maxDriveSpeed, x, y, turnSpeed);
        } else if (!this.stopped) {
            this.stopped = true;
            moveRobotX(maxDriveSpeed, 0, 0, 0);
        }
    }

    public void moveByPower(double power, double x,  double y) {
        moveByPower(power, x, y, 0);
    }

    /**
     * read the Robot heading directly from the IMU (in degrees)
     */
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

}
