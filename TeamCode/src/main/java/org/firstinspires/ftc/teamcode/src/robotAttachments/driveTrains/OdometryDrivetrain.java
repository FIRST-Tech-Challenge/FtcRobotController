package org.firstinspires.ftc.teamcode.src.robotAttachments.driveTrains;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.src.robotAttachments.odometry.OdometryGlobalCoordinatePosition;
import org.firstinspires.ftc.teamcode.src.robotAttachments.odometry.enums.FieldPoints;
import org.firstinspires.ftc.teamcode.src.robotAttachments.odometry.enums.OdometryDirections;
import org.firstinspires.ftc.teamcode.src.robotAttachments.sensors.RobotVoltageSensor;
import org.firstinspires.ftc.teamcode.src.utills.Executable;
import org.firstinspires.ftc.teamcode.src.utills.MiscUtills;

/**
 * Odometry Drivetrain Implements basic drive functions that can be inherited by other drive systems.
 */
public class OdometryDrivetrain extends BasicDrivetrain {
    /**
     * Internal Telemetry Object, allows debug information
     */
    private final Telemetry telemetry;
    /**
     * Internal Odometry Global Coordinate Position Object, it runs the localization algorithm in a separate thread
     */
    OdometryGlobalCoordinatePosition odometry;
    /**
     * A Lambda object that allows this class to check the stop requested condition of the OpMode
     */
    Executable<Boolean> _isStopRequested;
    /**
     * A Lambda object that allows this class to check that the OpMode is active
     */
    Executable<Boolean> _opModeIsActive;

    /**
     * A voltage sensor to monitor the robot voltage
     */
    RobotVoltageSensor voltageSensor;
    /**
     * accelerationDistance controls the distance (in inches) that the robot uses to accelerate to maximum speed
     */
    private final double accelerationDistance = 10.0D;

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
     * A constructor that takes a already initialized BasicDrivetrain object
     *
     * @param drivetrain      A already initialized BasicDrivetrain object
     * @param telemetry       Telemetry object from the OpMode
     * @param odometry        A Already Initialized OdometryGlobalCoordinatePosition object
     * @param isStopRequested A Executable object wrapped around OpMode.isStopRequested()
     * @param opmodeIsActive  A Executable object wrapped around OpMode.opModeIsActive()
     * @param voltageSensor   an already initialized voltage sensor
     */
    public OdometryDrivetrain(BasicDrivetrain drivetrain, Telemetry telemetry, OdometryGlobalCoordinatePosition odometry, Executable<Boolean> isStopRequested, Executable<Boolean> opmodeIsActive, RobotVoltageSensor voltageSensor) {
        super(drivetrain.front_right, drivetrain.front_left, drivetrain.back_right, drivetrain.back_left);
        this.telemetry = telemetry;
        this.odometry = odometry;
        this._isStopRequested = isStopRequested;
        this._opModeIsActive = opmodeIsActive;
        this.voltageSensor = voltageSensor;
    }

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
        moveToPosition(pos[0], pos[1], tolerance);
        this.stopAll();
    }

    /**
     * Determines the power to drive the motor at for the given distance away
     *
     * @param totalDistance   the total distance in inches, that the robot is expected to go
     * @param currentDistance the total distance in inches, that the robot has traveled
     * @return The power to drive the motor at in a range between -1 and 1
     */
    private double calculateLongDistancePower(double totalDistance, double currentDistance) {
        final double distanceTraveled = totalDistance - currentDistance;
        double power;


        if (distanceTraveled < accelerationDistance) {
            power = (-0.3 * Math.cos(distanceTraveled * (1.0 / accelerationDistance) * Math.PI)) + (0.7 * (normalVoltage / voltageSensor.getVoltage()));
        } else if (currentDistance < decelerationDistance) {
            power = (0.4 * Math.cos(distanceTraveled * (1.0 / decelerationDistance) * Math.PI)) + (0.6 * (normalVoltage / voltageSensor.getVoltage()));
        } else {
            return 1.0;
        }

        return MiscUtills.boundNumber(power);

    }

    /**
     * Determines the power to drive the motor at for the given distance away
     *
     * @param totalDistance   the total distance in inches, that the robot is expected to go
     * @param currentDistance the total distance in inches, that the robot has traveled
     * @return The power to drive the motor at in a range between -1 and 1
     */
    private double calculateShortDistancePower(double totalDistance, double currentDistance) {

        //return MiscUtills.boundNumber(0.5 * Math.sin((currentDistance * Math.PI) / (totalDistance * .75)) + (0.5 * (normalVoltage / voltageSensor.getVoltage())));
        return 0.8;

    }

    /**
     * A method for relative motion
     *
     * @param distance  The distance to travel
     * @param angle     The angle to travel at
     * @param tolerance The tolerance from the end point
     * @throws InterruptedException Throws if the OpMode ends during execution
     */
    public void move(double distance, double angle, double tolerance) throws InterruptedException {
        final double xComponent = distance * Math.sin(angle);
        final double yComponent = distance * Math.cos(angle);
        moveToPosition(odometry.returnRelativeXPosition() + xComponent, odometry.returnRelativeYPosition() + yComponent, tolerance);
    }

    /**
     * @param direction The distance to travel, specified by the {@link OdometryDirections} enum
     * @param distance  The angle to travel at
     * @param tolerance The tolerance from the end point
     * @throws InterruptedException Throws if the OpMode ends during execution
     */
    public void move(OdometryDirections direction, double distance, double tolerance) throws InterruptedException {
        Double angle = OdometryDirections.positionToAngle.get(direction);
        assert angle != null;
        move(distance, angle, tolerance);
    }

    /**
     * Moves the robot to the given position with the option for debug information
     *
     * @param x         X Value to move to
     * @param y         Y Value to move to
     * @param tolerance The distance the robot can be off from the given position
     * @throws InterruptedException Throws an exception if stop is requested during the move
     */
    public void moveToPosition(double x, double y, double tolerance) throws InterruptedException {
        try {
            moveToPositionWithCallBack(x, y, tolerance, () -> false); //Because the callback function always returns false, it cannot throw, thus it is safe to ignore this error
        } catch (OdometryMovementException ignored) {
        }
    }

    /**
     * Moves to the given position. Throws error if it is stopped for a time greater than millis.
     *
     * @param x         The x coordinate to go to
     * @param y         The y coordinate to go to
     * @param tolerance The tolerance for how close it must get
     * @param millis    The time in milliseconds that the robot should attempt to move
     * @throws InterruptedException      Throws if the OpMode ends during execution
     * @throws OdometryMovementException Stops Motors and Throws if the robot gets stuck and times out
     */
    public void moveToPositionWithDistanceTimeOut(double x, double y, double tolerance, long millis) throws InterruptedException, OdometryMovementException {
        final String coordinateString = x + " , " + y;
        double power, odometry_angle;

        double odometry_x = odometry.returnRelativeXPosition();
        double odometry_y = odometry.returnRelativeYPosition();

        double currentDistance = MiscUtills.distance(odometry_x, odometry_y, x, y);
        final double initialDistance = currentDistance;

        double longDistanceThreshold = decelerationDistance + accelerationDistance;
        final boolean longDistanceTravel = (initialDistance > longDistanceThreshold);
        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        double posA;
        double posB;
        double tooSmallOfDistance = millis / 500.0; // this distance is 1 inch for every second of millis


        while (currentDistance > tolerance && !isStopRequested() && opModeIsActive()) {
            timer.reset();
            posA = MiscUtills.distance(odometry.returnRelativeXPosition(), odometry.returnRelativeYPosition(), x, y);

            while (timer.milliseconds() < millis) {
                odometry_x = odometry.returnRelativeXPosition(); //odometry x
                odometry_y = odometry.returnRelativeYPosition(); //odometry y
                currentDistance = MiscUtills.distance(odometry_x, odometry_y, x, y); //currentDistance value
                odometry_angle = MiscUtills.getAngle(odometry_x, odometry_y, x, y, odometry.returnOrientation()); //angle


                if (longDistanceTravel) {
                    power = this.calculateLongDistancePower(initialDistance, currentDistance);
                } else {
                    power = this.calculateShortDistancePower(initialDistance, currentDistance);
                }

                // if current position does not decrease by a certain amount within a certain time, make timeout true


                if (this.debug) {
                    telemetry.addData("Moving to", coordinateString);
                    telemetry.addData("currentDistance", currentDistance);
                    telemetry.addData("angle", odometry_angle);
                    telemetry.addData("Moving?", (currentDistance > tolerance && !isStopRequested() && opModeIsActive()));
                    telemetry.addData("X Pos", odometry_x);
                    telemetry.addData("Y Pos", odometry_y);
                    telemetry.addData("Power", power);
                    telemetry.addData("Long Distance Mode = ", longDistanceTravel);
                    telemetry.update();
                }

                strafeAtAngle(odometry_angle, power);
            }

            posB = MiscUtills.distance(odometry.returnRelativeXPosition(), odometry.returnRelativeYPosition(), x, y);

            if (posA - posB < tooSmallOfDistance) {
                stopAll();
                throw new OdometryMovementException("Timeout");
            }

        }
        stopAll();
    }

    /**
     * Moves to the given position and errors out if the time elapsed in seconds is greater than timeout
     *
     * @param x         The x coordinate to go to
     * @param y         The y coordinate to go to
     * @param tolerance The tolerance for how close it must get
     * @param timeout   The time in seconds that the movement may take
     * @throws InterruptedException      Throws if the OpMode ends during execution
     * @throws OdometryMovementException Stops Motors and Throws if the movement time exceeds the provided value of timeout
     */
    public void moveToPositionWithTimeOut(double x, double y, double tolerance, double timeout) throws InterruptedException, OdometryMovementException {
        ElapsedTime t = new ElapsedTime();
        Executable<Boolean> e = () -> ((t.milliseconds() / 1000.0) > timeout);
        moveToPositionWithCallBack(x, y, tolerance, e);
    }

    /**
     * Moves to the given position and errors out if the voltage falls to far
     *
     * @param x             The x coordinate to go to
     * @param y             The y coordinate to go to
     * @param tolerance     The tolerance for how close it must get
     * @param consoleOutput A boolean to toggle debug information
     * @throws InterruptedException      Throws if the OpMode ends during execution
     * @throws OdometryMovementException Stops Motors and Throws if the voltage falls to 2 volts less than the starting voltage
     */
    public void moveToPositionWithVoltageSpike(double x, double y, double tolerance, boolean consoleOutput) throws InterruptedException, OdometryMovementException {
        final double initialVoltage = voltageSensor.getVoltage();
        Executable<Boolean> e = () -> voltageSensor.getVoltage() < (initialVoltage - 2);
        moveToPositionWithCallBack(x, y, tolerance, e);
    }

    /**
     * Moves to the given position. Errors out if callBack returns true
     *
     * @param x         The x coordinate to go to
     * @param y         The y coordinate to go to
     * @param tolerance The tolerance for how close it must get
     * @param callBack  A Lambda function, if it returns true, this throws a {@link OdometryMovementException}
     * @throws InterruptedException      Throws if the OpMode ends during execution
     * @throws OdometryMovementException Stops Motors and Throws if callBack returns true
     */
    public void moveToPositionWithCallBack(double x, double y, double tolerance, Executable<Boolean> callBack) throws InterruptedException, OdometryMovementException {
        final String coordinateString = x + " , " + y;
        double power, odometry_angle;

        double odometry_x = odometry.returnRelativeXPosition();
        double odometry_y = odometry.returnRelativeYPosition();

        double currentDistance = MiscUtills.distance(odometry_x, odometry_y, x, y);
        final double initialDistance = currentDistance;

        double longDistanceThreshold = decelerationDistance + accelerationDistance;
        final boolean longDistanceTravel = (initialDistance > longDistanceThreshold);

        while (currentDistance > tolerance && !isStopRequested() && opModeIsActive()) {
            odometry_x = odometry.returnRelativeXPosition(); //odometry x
            odometry_y = odometry.returnRelativeYPosition(); //odometry y
            currentDistance = MiscUtills.distance(odometry_x, odometry_y, x, y); //currentDistance value
            odometry_angle = MiscUtills.getAngle(odometry_x, odometry_y, x, y, odometry.returnOrientation()); //angle

            if (longDistanceTravel) {
                power = this.calculateLongDistancePower(initialDistance, currentDistance);
            } else {
                power = this.calculateShortDistancePower(initialDistance, currentDistance);
            }

            if (this.debug) {
                telemetry.addData("Moving to", coordinateString);
                telemetry.addData("currentDistance", currentDistance);
                telemetry.addData("angle", odometry_angle);
                telemetry.addData("Moving?", (currentDistance > tolerance && !isStopRequested() && opModeIsActive()));
                telemetry.addData("X Pos", odometry_x);
                telemetry.addData("Y Pos", odometry_y);
                telemetry.addData("Power", power);
                telemetry.addData("Long Distance Mode = ", longDistanceTravel);
                telemetry.update();
            }

            strafeAtAngle(odometry_angle, power);

            if (callBack.call()) {
                stopAll();
                throw new OdometryMovementException("Callback Returned True");
            }

        }
        stopAll();
    }

    /**
     * Moves to position while strafing at angle
     *
     * @param x         The x-coordinate to move to
     * @param y         The y-coordinate to move to
     * @param theta     The angle to turn to, relative to the field
     * @param tolerance The tolerance in inches that is good enough
     * @throws InterruptedException Throws if stop is requested during this time
     */
    public void moveToPosition(double x, double y, double theta, double tolerance) throws InterruptedException {
        double power, odometry_angle = 0;
        final String args = "moveToPosition(" + x + ", " + y + ", " + theta + ", " + tolerance + ");";
        final String coordinateString = x + " , " + y;

        double odometry_x = odometry.returnRelativeXPosition();
        double odometry_y = odometry.returnRelativeYPosition();

        double currentDistance = MiscUtills.distance(odometry_x, odometry_y, x, y);
        final double initialDistance = currentDistance;

        double longDistanceThreshold = decelerationDistance + accelerationDistance;
        final boolean longDistanceTravel = (initialDistance > longDistanceThreshold);
        while (currentDistance > tolerance && !isStopRequested() && opModeIsActive()) {
            if (longDistanceTravel) {
                power = this.calculateLongDistancePower(initialDistance, currentDistance);
            } else {
                power = this.calculateShortDistancePower(initialDistance, currentDistance);
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
                telemetry.addData("Long Distance Mode = ", longDistanceTravel);
                telemetry.update();
            }

            odometry_x = odometry.returnRelativeXPosition(); //odometry x
            odometry_y = odometry.returnRelativeYPosition(); //odometry y
            currentDistance = MiscUtills.distance(odometry_x, odometry_y, x, y); //currentDistance value
            odometry_angle = MiscUtills.getAngle(odometry_x, odometry_y, x, y, odometry.returnOrientation()); //angle

            strafeAtAngleWhileTurn(odometry_angle, theta, power);

        }
        stopAll();
    }

    /**
     * Strafes at the provided angle relative to the robot, and turns to the given turnAngle
     *
     * @param angle     The angle to strafe at
     * @param turnAngle The angle to turn to
     * @param power     The power to turn at
     */
    private void strafeAtAngleWhileTurn(double angle, double turnAngle, double power) {
        power = MiscUtills.boundNumber(power);
        double power1;
        double power2;
        double power3;
        double power4;

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
