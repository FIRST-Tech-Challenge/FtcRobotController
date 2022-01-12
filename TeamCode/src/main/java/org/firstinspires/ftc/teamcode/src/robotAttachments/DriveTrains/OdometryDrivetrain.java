package org.firstinspires.ftc.teamcode.src.robotAttachments.DriveTrains;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.src.Utills.Executable;
import org.firstinspires.ftc.teamcode.src.Utills.MiscUtills;
import org.firstinspires.ftc.teamcode.src.robotAttachments.Sensors.RobotVoltageSensor;
import org.firstinspires.ftc.teamcode.src.robotAttachments.odometry.FieldPoints;
import org.firstinspires.ftc.teamcode.src.robotAttachments.odometry.OdometryGlobalCoordinatePosition;

import java.util.HashMap;

/**
 * Odometry Drivetrain Implements basic drive functions that can be inherited by other drive systems.
 */
public class OdometryDrivetrain extends BasicDrivetrain {
    /**
     * Internal Telemetry Object, allows debug information
     */
    Telemetry telemetry;
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
     * A empty constructor for subclassing
     */
    protected OdometryDrivetrain() {
        super();
    }

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
     * @param voltageSensor an already initialized voltage sensor
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
     * Determines the distance between two points
     *
     * @param x1 the x-value of the first point
     * @param y1 the y-value of the first point
     * @param x2 the x-value of the second point
     * @param y2 the y-value of the second point
     * @return The distance between two points
     */
    private static double distance(double x1, double y1, double x2, double y2) {
        return Math.sqrt((y2 - y1) * (y2 - y1) + (x2 - x1) * (x2 - x1));
    }

    /**
     * This is used to get the angle between two points
     *
     * @param rx       The robot x position
     * @param ry       Robot Y Position
     * @param x        X Position to go to
     * @param y        Y position to go to
     * @param robotRot The orientation of the robot
     * @return The heading the point is from the robot
     */
    private static double getAngle(double rx, double ry, double x, double y, double robotRot) {
        double angle;
        x = x - rx;
        y = y - ry;
        angle = Math.toDegrees(Math.atan2(x, y));
        return ((angle - robotRot) % 360);
    }

    /**
     * Turns the robot to the given angle relative to the odometry zero angle.
     *
     * @param turnAngle The angle to turn to
     * @param power     The power to turn at
     * @throws InterruptedException This exception is thrown to stop the OpMode in response to the stop button
     */
    public void turnTo(double turnAngle, double power) throws InterruptedException {
        double position = odometry.returnOrientation();

        // the following calculation determines the value of the angle between the current position and the desired position in a counterclockwise rotation/left turn
        if (((360 - turnAngle) + position) % 360 > 180) {
            while (((360 - turnAngle) + odometry.returnOrientation()) % 360 > 180) {
                if (!isStopRequested() && opModeIsActive()) {
                    this.turnLeft(power);
                } else {
                    break;
                }
            }
        } else {
            // while the left turn angle value is less than or equal to 180, turn left
            while (((360 - turnAngle) + odometry.returnOrientation()) % 360 <= 180) {
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
     * This determines what way the robot will turn based on given angle
     *
     * @param turnAngle The angle to turn towards
     * @param power     The power to turn at
     */
    private void turnWithStrafe(double turnAngle, double power) {
        // this method is only meant for use in moveToPositionWithTurn
        if (((360 - turnAngle) + odometry.returnOrientation()) % 360 > 180) {
            turnRight(power);
        } else {
            turnLeft(power);
        }

    }

    /**
     * This wraps the Executable _isStopRequested
     *
     * @return it returns false if the OpMode stop is not requested
     * @throws InterruptedException Throws if stop is requested
     */
    boolean isStopRequested() throws InterruptedException {
        if (_isStopRequested.call()) {
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
     * Moves the robot to the provided position
     *
     * @param x         X Value to move to
     * @param y         Y Value to move to
     * @param tolerance The distance the robot can be off from the given position
     * @throws InterruptedException Throws an exception if stop is requested during the move
     */
    public void moveToPosition(double x, double y, double tolerance) throws InterruptedException {
        moveToPosition(x, y, tolerance, false);
        this.stopAll();
    }

    /**
     * Moves the robot to the provided position Enum
     *
     * @param position  a hashmap value referencing the 2 value array of the position
     * @param tolerance The distance the robot can be off from the given position
     * @throws InterruptedException
     */
    public void moveToPosition(FieldPoints position, double tolerance) throws InterruptedException {

        moveToPosition(FieldPoints.positionsAndPoints.get(position)[0], FieldPoints.positionsAndPoints.get(position)[1], tolerance, false);
        this.stopAll();
    }

    public void moveToPosition(FieldPoints position, double tolerance, boolean debug) throws InterruptedException {

        moveToPosition(FieldPoints.positionsAndPoints.get(position)[0], FieldPoints.positionsAndPoints.get(position)[1], tolerance, debug);
        this.stopAll();
    }

    /**
     * Precisely moves the robot to the given position
     *
     * @param x         The x position to move to
     * @param y         The y position to move to
     * @param tolerance The tolerence for the movement
     * @throws InterruptedException Throws an exception if stop is requested during the move
     */
    private void preciseMovement(double x, double y, double tolerance) throws InterruptedException {
        double power = 0.1;
        final String s = x + " , " + y;
        while (distance(odometry.returnRelativeXPosition(), odometry.returnRelativeYPosition(), x, y) > tolerance && !isStopRequested()) {
            telemetry.addData("Moving to", s);
            telemetry.update();

            strafeAtAngle(getAngle(odometry.returnRelativeXPosition(), odometry.returnRelativeXPosition(), x, y, odometry.returnOrientation()), 0.5);

        }
        this.stopAll();
    }

    /**
     * These four variables configure the speed at which the robot moves during the odometry movement functions
     * accelerationDistance controls the distance (in inches) that the robot uses to accelerate to maximum speed
     * decelerationDistance controls the distance (in inches) that the robot uses to decelerate from maximum speed
     * LongDistanceThreshold controls the distance at which the robot will swich from long to short distance mode
     * normalVoltage is the voltage the robot is expected to operate at. If the voltage goes lower, the power returned is higher to compensate and visa versa
     */
    private final double accelerationDistance = 10.0D;
    private final double decelerationDistance = 20.0D;
    private final double LongDistanceThreshold = decelerationDistance + accelerationDistance;
    private final double normalVoltage = 12.0D;

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
        final double distancePercentage = (totalDistance - currentDistance) / totalDistance;
        //return MiscUtills.boundNumber((-1.0 / 4) * Math.cos(distancePercentage) + (0.5 * (normalVoltage / voltageSensor.getVoltage())));
        return 0.5 * (12 / voltageSensor.getVoltage());
    }

    public void move(double distance, double angle, double tolerance) throws InterruptedException {
        final double xComponent = distance * Math.sin(angle);
        final double yComponent = distance * Math.cos(angle);
        moveToPosition(odometry.returnRelativeXPosition() + xComponent, odometry.returnRelativeYPosition() + yComponent, tolerance);
    }

    public void move(OdometryDirections direction, double distance, double tolerance) throws InterruptedException {
        move(distance, OdometryDirections.positionToAngle.get(direction), tolerance);
    }

    /**
     * Moves the robot to the given position with the option for debug information
     *
     * @param x             X Value to move to
     * @param y             Y Value to move to
     * @param tolerance     The distance the robot can be off from the given position
     * @param consoleOutput Prints debug info to the console for debugging, is slower and less accurate
     * @throws InterruptedException Throws an exception if stop is requested during the move
     */
    public void moveToPosition(double x, double y, double tolerance, boolean consoleOutput) throws InterruptedException {
        final String coordinateString = x + " , " + y;
        double power, odometry_angle;

        double odometry_x = odometry.returnRelativeXPosition();
        double odometry_y = odometry.returnRelativeYPosition();

        double currentDistance = distance(odometry_x, odometry_y, x, y);
        final double initialDistance = currentDistance;

        final boolean longDistanceTravel = (initialDistance > LongDistanceThreshold);


        while (currentDistance > tolerance && !isStopRequested() && opModeIsActive()) {
            odometry_x = odometry.returnRelativeXPosition(); //odometry x
            odometry_y = odometry.returnRelativeYPosition(); //odometry y
            currentDistance = distance(odometry_x, odometry_y, x, y); //currentDistance value
            odometry_angle = getAngle(odometry_x, odometry_y, x, y, odometry.returnOrientation()); //angle

            if (longDistanceTravel) {
                power = this.calculateLongDistancePower(initialDistance, currentDistance);
            } else {
                power = this.calculateShortDistancePower(initialDistance, currentDistance);
            }

            if (consoleOutput) {
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
        stopAll();
    }


    public enum OdometryDirections {
        Forward,
        Backward,
        Right,
        Left;

        public static final HashMap<OdometryDirections, Double> positionToAngle = new HashMap<OdometryDirections, Double>() {{
            put(OdometryDirections.Forward, 0D);
            put(OdometryDirections.Backward, 180D);
            put(OdometryDirections.Right, 90D);
            put(OdometryDirections.Left, 270D);
        }};
    }

    /**
     * Moves to position and turns over the movement
     *
     * @param x             X Value to move to
     * @param y             Y Value to move to
     * @param rotate        How much the robot is to rotate over the movement
     * @param tolerance     The distance the robot can be off from the given position
     * @param consoleOutput Prints debug info to the console for debugging, is slower and less accurate
     * @throws InterruptedException Throws an exception if stop is requested during the move
     */
    public void moveToPositionWithTurn(double x, double y, double rotate, double tolerance, boolean consoleOutput) throws InterruptedException {
        final String s = x + " , " + y;
        double power = 0;
        //by setting distance to max value, we make sure that the loop will execute once
        //by recalculating distance in the loop rather than in the while parenthesises, we remove one distance() call
        double distance = Double.MAX_VALUE;
        double odometry_angle;
        double odometry_x;
        double odometry_y;

        while (distance > tolerance && !isStopRequested() && opModeIsActive()) {
            //By calculating the values here once in this loop and declaring the variables above, we minimize the number
            //of memory allocation calls and the number of variable calculations.
            odometry_x = odometry.returnRelativeXPosition(); //odometry x
            odometry_y = odometry.returnRelativeYPosition(); //odometry y
            distance = distance(odometry_x, odometry_y, x, y); //distance value
            odometry_angle = getAngle(odometry_x, odometry_y, x, y, odometry.returnOrientation()); //angle

            /*The next if-else block takes the distance from target and
             sets the power variable to odometry_angle power following the function
             @param zeroPoint is the point where the robot goes at 1
             power = 0.8/zeroPoint(distance) + 0.2
             if the distance is greater than 24 in or ~2 ft, robot moves at power of 1
             */
            final double zeroPoint = 24;
            if (distance > zeroPoint) {
                power = 1;
            } else {
                power = ((0.9 / zeroPoint) * distance) + 0.2;
            }
            //power = Math.abs(power);


            if (consoleOutput) {

                telemetry.addData("Moving to", s);
                telemetry.addData("distance", distance);
                telemetry.addData("angle", odometry_angle);
                telemetry.addData("Moving?", distance > tolerance);
                telemetry.addData("X Pos", odometry_x);
                telemetry.addData("Y Pos", odometry_y);
                telemetry.addData("Power", power);
                telemetry.update();

            }
            /* strafeAtAngle and turnWithStrafe set motor power values in accordance with their desired
            movement. Since they are both in a while loop and do not call stopAll() within their declarations,
            these movements alternate rapidly to create a simultaneous strafe and turn movement
             */
            strafeAtAngle(odometry_angle, power);
            turnWithStrafe(rotate, power);


        }
        stopAll();
    }

    /**
     * A debug method
     *
     * @return All odometry raw encoder counts
     */
    public int[] getOdometryRaw() {
        return odometry.returnRaw();
    }

    /**
     * A debug method
     *
     * @return returns the right encoder position
     */
    public int returnRightEncoderPosition() {
        return odometry.returnRightEncoderPosition();
    }

    /**
     * A debug method
     *
     * @return returns left encoder position
     */
    public int returnLeftEncoderPosition() {
        return odometry.returnLeftEncoderPosition();
    }

    /**
     * A debug method
     *
     * @return returns the right encoder position
     */
    public int returnHorizontalEncoderPosition() {
        return odometry.returnHorizontalEncoderPosition();
    }

}
