package org.firstinspires.ftc.teamcode.src.robotAttachments.driveTrains;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.src.robotAttachments.odometry.OdometryGlobalCoordinatePosition;
import org.firstinspires.ftc.teamcode.src.robotAttachments.odometry.enums.FieldPoints;
import org.firstinspires.ftc.teamcode.src.robotAttachments.sensors.RobotVoltageSensor;
import org.firstinspires.ftc.teamcode.src.utills.Executable;
import org.firstinspires.ftc.teamcode.src.utills.MiscUtills;

/**
 * Odometry Drivetrain Implements more advanced drive functions that can be inherited by other drive systems.
 */
public class OdometryDrivetrain extends BasicDrivetrain {

    /**
     * Internal Telemetry Object, allows debug information
     */
    private final Telemetry telemetry;

    /**
     * Internal Odometry Global Coordinate Position Object, it runs the localization algorithm in a separate thread
     */
    final OdometryGlobalCoordinatePosition odometry;

    /**
     * A Lambda object that allows this class to check the stop requested condition of the OpMode
     */
    final Executable<Boolean> _isStopRequested;

    /**
     * A Lambda object that allows this class to check that the OpMode is active
     */
    final Executable<Boolean> _opModeIsActive;

    /**
     * A voltage sensor to monitor the robot voltage
     */
    final RobotVoltageSensor voltageSensor;

    /**
     * accelerationDistance controls the distance (in inches) that the robot uses to accelerate to maximum speed
     */
    private final double accelerationDistance = 10.0D;

    /**
     * decelerationDistance controls the distance (in inches) that the robot uses to decelerate from maximum speed
     */
    private final double decelerationDistance = 20.0D;

    /**
     * normalVoltage is the voltage the robot is expected to operate at. If the voltage goes lower, the power returned is higher to compensate and visa versa
     */
    private final double normalVoltage = 12.0D;

    /**
     * A internal variable to control debug printing, true for on, false for off
     */
    private boolean debug = false;

    //-Utility Methods-------------------------------------------------------------------------------------------

    /**
     * A constructor that takes already initialized DcMotor Objects, Telemetry, Odometry,and Lambda objects
     *
     * @param front_right     A DcMotor object tied to the front right motor
     * @param front_left      A DcMotor object tied to the front left motor
     * @param back_right      A DcMotor object tied to the back right motor
     * @param back_left       A DcMotor object tied to the back left motor
     * @param telemetry       Telemetry object from the OpMode
     * @param odometry        A Already Initialized OdometryGlobalCoordinatePosition object
     * @param isStopRequested A Executable object wrapped around OpMode.isStopRequested()
     * @param opmodeIsActive  A Executable object wrapped around OpMode.opModeIsActive()
     * @param voltageSensor   an already initialized voltage sensor
     */
    public OdometryDrivetrain(DcMotor front_right, DcMotor front_left, DcMotor back_right, DcMotor back_left, Telemetry telemetry, OdometryGlobalCoordinatePosition odometry, Executable<Boolean> isStopRequested, Executable<Boolean> opmodeIsActive, RobotVoltageSensor voltageSensor) {
        super(front_right, front_left, back_right, back_left);
        this.telemetry = telemetry;
        this.odometry = odometry;
        this._isStopRequested = isStopRequested;
        this._opModeIsActive = opmodeIsActive;
        this.voltageSensor = voltageSensor;
    }

    /**
     * This wraps the Executable _isStopRequested
     *
     * @return it returns false if the OpMode stop is not requested
     * @throws InterruptedException Throws if stop is requested
     */
    boolean isStopRequested() throws InterruptedException {
        if (_isStopRequested.call() || Thread.currentThread().isInterrupted()) {
            throw new InterruptedException();
        }
        return false;
    }

    /**
     * This wraps the Executable _opModeIsActive
     *
     * @return it returns true if the OpMode is active, returns false otherwise
     */
    boolean opModeIsActive() {
        return _opModeIsActive.call();
    }

    /**
     * Turns the debug information on
     */
    public void debugOn() {
        this.debug = true;
    }

    /**
     * Turns the debug information off
     */
    public void debugOff() {
        this.debug = false;
    }

    //-Movement Methods---------------------------------------------------------------------------------------------

    /**
     * Turns the robot to the given angle relative to the odometry zero angle.
     *
     * @param angle The angle to turn to
     * @param power The power to turn at
     * @throws InterruptedException This exception is thrown to stop the OpMode in response to the stop button
     */
    public void turnTo(double angle, double power) throws InterruptedException {
        double startingAngle = odometry.returnOrientation();

        // the following calculation determines the value of the angle between the current startingAngle and the desired startingAngle in a counterclockwise rotation/left turn
        if (((360 - angle) + startingAngle) % 360 > 180) {
            while (((360 - angle) + odometry.returnOrientation()) % 360 > 180) {
                if (!isStopRequested() && opModeIsActive()) {
                    this.turnLeft(power);
                } else {
                    break;
                }
            }
        } else {
            // while the left turn angle value is less than or equal to 180, turn left
            while (((360 - angle) + odometry.returnOrientation()) % 360 <= 180) {
                if (!isStopRequested() && opModeIsActive()) {
                    this.turnRight(power);
                } else {
                    break;
                }
            }
        }
        stopAll();
    }

    //-Move To Position Methods----------------------------------------------------------------------------------------------

    /**
     * Moves the robot to the provided position Enum
     *
     * @param position  a hashmap value referencing the 2 value array of the position
     * @param tolerance The distance the robot can be off from the given position
     * @throws InterruptedException Throws if the opMode is stopped
     */
    public void moveToPosition(FieldPoints position, double tolerance) throws InterruptedException {
        double[] pos = FieldPoints.positionsAndPoints.get(position);
        assert pos != null;
        moveToPosition(pos[0], pos[1], odometry.returnOrientation(), tolerance);
        this.stopAll();
    }

    /**
     * Moves the robot to the provided position Enum
     *
     * @param position  a hashmap value referencing the 2 value array of the position
     * @param tolerance The distance the robot can be off from the given position
     * @param theta     The angle to turn to
     * @throws InterruptedException Throws if the opMode is stopped
     */
    public void moveToPosition(FieldPoints position, double theta, double tolerance) throws InterruptedException {
        double[] pos = FieldPoints.positionsAndPoints.get(position);
        assert pos != null;
        moveToPosition(pos[0], pos[1], theta, tolerance);
        this.stopAll();
    }

    /**
     * Moves the robot to the given (x,y) position
     *
     * @param x         X Value to move to
     * @param y         Y Value to move to
     * @param tolerance The distance the robot can from the given position (in inches)
     * @throws InterruptedException Throws an exception if stop is requested during the move
     */
    public void moveToPosition(double x, double y, double tolerance) throws InterruptedException {
        moveToPosition(x, y, odometry.returnOrientation(), tolerance);
    }

    /**
     * Moves the robot to the given (x,y) position and turns to the angle (theta)
     *
     * @param x         The x-coordinate to move to
     * @param y         The y-coordinate to move to
     * @param theta     The angle to turn to, relative to the field
     * @param tolerance The tolerance for how close it must get (in inches)
     * @throws InterruptedException Throws if stop is requested during this time
     */
    public void moveToPosition(double x, double y, double theta, double tolerance) throws InterruptedException {
        try {
            moveToPosition(x, y, theta, tolerance, () -> false);
        } catch (OdometryMovementException ignored) {
        }
    }

    /**
     * Moves the robot to the given (x,y) position and turns to the given angle (theta). Errors out if callBack returns true
     *
     * @param x         The x coordinate to go to
     * @param y         The y coordinate to go to
     * @param tolerance The tolerance for how close it must get (in inches)
     * @param theta     The angle (relative to the field) to turn to during the movement
     * @param callBack  A Lambda function, if it returns true, this throws a {@link OdometryMovementException}
     * @throws InterruptedException      Throws if the OpMode ends during execution
     * @throws OdometryMovementException Stops Motors and Throws if callBack returns true
     */
    public void moveToPosition(double x, double y, double theta, double tolerance, Executable<Boolean> callBack) throws InterruptedException, OdometryMovementException {
        double power, odometry_angle = 0;
        final String args = "moveToPosition(" + x + ", " + y + ", " + theta + ", " + tolerance + ")\n";
        final String coordinateString = x + " , " + y;

        double odometry_x = odometry.returnRelativeXPosition();
        double odometry_y = odometry.returnRelativeYPosition();

        double currentDistance = MiscUtills.distance(odometry_x, odometry_y, x, y);

        while (currentDistance > tolerance && !isStopRequested() && opModeIsActive()) {

            /*The next if-else block takes the distance from target and
             sets the power variable to odometry_angle power following the function
             @param zeroPoint is the point where the robot goes at 1
             power = 0.8/zeroPoint(distance) + 0.2
             if the distance is greater than 24 in or ~2 ft, robot moves at power of 1
             */
            final double zeroPoint = 24;
            if (currentDistance > zeroPoint) {
                power = 1;
            } else {
                power = ((0.9 / zeroPoint) * currentDistance) + 0.2;
            }
            if (this.debug) {
                telemetry.addData("Function", args);
                telemetry.addData("Moving to", coordinateString);
                telemetry.addData("currentDistance", currentDistance);
                telemetry.addData("angle", odometry_angle);
                telemetry.addData("Moving?", (currentDistance > tolerance && !isStopRequested() && opModeIsActive()));
                telemetry.addData("X Pos", odometry_x);
                telemetry.addData("Y Pos", odometry_y);
                telemetry.addData("Power", power);
                telemetry.update();
            }

            odometry_x = odometry.returnRelativeXPosition(); //odometry x
            odometry_y = odometry.returnRelativeYPosition(); //odometry y
            currentDistance = MiscUtills.distance(odometry_x, odometry_y, x, y); //currentDistance value
            odometry_angle = MiscUtills.getAngle(odometry_x, odometry_y, x, y, odometry.returnOrientation()); //angle

            if (callBack.call()) {
                if (this.debug) {
                    RobotLog.addGlobalWarningMessage("Failed Odometry Movement: " + args);
                }
                stopAll();
                throw new OdometryMovementException("Callback Returned True");
            }

            strafeAtAngle(odometry_angle, power);

        }
        stopAll();
    }

    //-Special Forms of MoveToPosition----------------------------------------------------------------------------------------------------------

    /**
     * Moves the robot to the given (x,y) position. Throws error if it is stopped for a time greater than millis.
     *
     * @param x         The x coordinate to go to
     * @param y         The y coordinate to go to
     * @param tolerance The tolerance for how close it must get
     * @param millis    The time in milliseconds that the robot should attempt to move
     * @param theta     The angle (relative to the field) to turn to during the movement
     * @throws InterruptedException      Throws if the OpMode ends during execution
     * @throws OdometryMovementException Stops Motors and Throws if the robot gets stuck and times out
     */
    public void moveToPositionWithDistanceTimeOut(double x, double y, double theta, double tolerance, long millis) throws InterruptedException, OdometryMovementException {
        final ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        final double[] positionBeforeTimeLoop = {0}; //These are arrays to make the compiler happy. Treat them as a normal double
        final double[] positionAfterTimeLoop = {Double.MAX_VALUE}; //These are arrays to make the compiler happy. Treat them as a normal double
        final double tooSmallOfDistance = millis / 500.0; // this travels ~2 inches for every 1000 millis

        Executable<Boolean> e = () -> {

            if (timer.milliseconds() >= millis) {
                positionBeforeTimeLoop[0] = positionAfterTimeLoop[0];
                positionAfterTimeLoop[0] = MiscUtills.distance(odometry.returnRelativeXPosition(), odometry.returnRelativeYPosition(), x, y);
                double traveledDistance = Math.abs(positionBeforeTimeLoop[0] - positionAfterTimeLoop[0]);
                if (traveledDistance < tooSmallOfDistance) {
                    return true;
                }
                timer.reset();
            }
            return false;
        };

        moveToPosition(x, y, theta, tolerance, e);

    }

    /**
     * Moves the robot to the given (x,y) position. Errors out if the time elapsed is greater than timeout.
     *
     * @param x         The x coordinate to go to.
     * @param y         The y coordinate to go to.
     * @param tolerance The tolerance for how close it must get (in inches).
     * @param timeout   The time (in seconds) before the method errors out.
     * @param theta     The angle (relative to the field) to turn to during the movement
     * @throws InterruptedException      Throws if the OpMode ends during execution.
     * @throws OdometryMovementException Stops Motors and Throws if the movement time exceeds the provided value of timeout.
     */
    public void moveToPositionWithTimeOut(double x, double y, double theta, double tolerance, double timeout) throws InterruptedException, OdometryMovementException {
        ElapsedTime t = new ElapsedTime();
        Executable<Boolean> e = () -> ((t.milliseconds() / 1000.0) > timeout);
        moveToPosition(x, y, theta, tolerance, e);
    }

    /**
     * Moves the robot to the given (x,y) position. Errors out if the voltage falls by 2V
     *
     * @param x         The x coordinate to go to
     * @param y         The y coordinate to go to
     * @param tolerance The tolerance for how close it must get
     * @param theta     The angle (relative to the field) to turn to during the movement
     * @throws InterruptedException      Throws if the OpMode ends during execution
     * @throws OdometryMovementException Stops Motors and Throws if the voltage falls to 2 volts less than the starting voltage
     */
    public void moveToPositionWithVoltageDrop(double x, double y, double theta, double tolerance) throws InterruptedException, OdometryMovementException {
        final double initialVoltage = voltageSensor.getVoltage();
        Executable<Boolean> e = () -> voltageSensor.getVoltage() < (initialVoltage - 2);
        moveToPosition(x, y, theta, tolerance, e);
    }

    /**
     * Strafes at the provided angle relative to the robot, and turns to the given turnAngle
     *
     * @param angle     The angle to strafe at
     * @param turnAngle The angle to turn to
     * @param power     The power to turn at
     */
    private void strafeAtAngleWhileTurn(double angle, double turnAngle, double power) {

        final boolean factorInAngleTurn = false;
        power = MiscUtills.boundNumber(power);
        double power1;
        double power2;
        double power3 = 0;
        double power4 = 0;

        angle = angle % 360;

        power1 = -Math.cos(Math.toRadians(angle + 45.0)); //power 1 is front right and back left
        power2 = -Math.cos(Math.toRadians(angle - 45)); // power 2 is front right and back left

        power1 = power * power1;
        power2 = power * power2;

        double degreesOff = ((odometry.returnOrientation() - turnAngle) % 360);
        double tmp;
        if (degreesOff < 180) {
            tmp = MiscUtills.map(degreesOff, 0, 180, 0, 1);
        } else {
            tmp = MiscUtills.map(degreesOff, 180, 360, -1, 0);
        }
        power3 = -tmp;
        power4 = tmp;


        front_right.setPower(power1 + power3);
        back_left.setPower(power1 + power4);

        front_left.setPower(power2 + power4);
        back_right.setPower(power2 + power3);

    }

}
