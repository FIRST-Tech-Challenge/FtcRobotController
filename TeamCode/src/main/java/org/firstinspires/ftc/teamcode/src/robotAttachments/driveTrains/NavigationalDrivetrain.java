package org.firstinspires.ftc.teamcode.src.robotAttachments.driveTrains;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.LocalizationAlgorithm;
import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.navigationExceptions.MovementException;
import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.navigationWarnings.MovementWarning;
import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.odometry.enums.FieldPoints;
import org.firstinspires.ftc.teamcode.src.robotAttachments.sensors.RobotVoltageSensor;
import org.firstinspires.ftc.teamcode.src.utills.Executable;
import org.firstinspires.ftc.teamcode.src.utills.MiscUtils;

/**
 * NavigationalDrivetrain implements more advanced drive functions that can be inherited by other drive systems.
 */
public class NavigationalDrivetrain extends BasicDrivetrain {

    /**
     * Internal Odometry Global Coordinate Position Object, it runs the localization algorithm in a separate thread
     */
    final LocalizationAlgorithm gps;
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
     * Internal Telemetry Object, allows debug information
     */
    private final Telemetry telemetry;
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

    /**
     * If this is true, the robot will turn while it strafes. If it is false, the robot will ignore theta and just strafe towards it's wanted point
     * This is to keep backwards compatibility
     * This can be toggled with NavigationalDrivetrain.setTurnWhileStrafe
     */
    private boolean turnWhileStrafe = false;

    //-Utility Methods-------------------------------------------------------------------------------------------

    /**
     * A constructor that takes already initialized DcMotor Objects, Telemetry, Odometry,and Lambda objects
     *
     * @param front_right     A DcMotor object tied to the front right motor
     * @param front_left      A DcMotor object tied to the front left motor
     * @param back_right      A DcMotor object tied to the back right motor
     * @param back_left       A DcMotor object tied to the back left motor
     * @param telemetry       Telemetry object from the OpMode
     * @param gps             A Already Initialized ThreeWheelOdometry object
     * @param isStopRequested A Executable object wrapped around OpMode.isStopRequested()
     * @param opmodeIsActive  A Executable object wrapped around OpMode.opModeIsActive()
     * @param voltageSensor   an already initialized voltage sensor
     */
    public NavigationalDrivetrain(DcMotor front_right, DcMotor front_left, DcMotor back_right, DcMotor back_left, Telemetry telemetry, LocalizationAlgorithm gps, Executable<Boolean> isStopRequested, Executable<Boolean> opmodeIsActive, RobotVoltageSensor voltageSensor) {
        super(front_right, front_left, back_right, back_left);
        this.telemetry = telemetry;
        this.gps = gps;
        this._isStopRequested = isStopRequested;
        this._opModeIsActive = opmodeIsActive;
        this.voltageSensor = voltageSensor;
    }

    /**
     * A setter method for the internal turnWhileStrafe variable
     *
     * @param value The value to set the internal turnWhileStrafe variable to
     */
    public void setTurnWhileStrafe(boolean value) {
        this.turnWhileStrafe = value;
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
        double startingAngle = gps.getRot();

        // the following calculation determines the value of the angle between the current startingAngle and the desired endingAngle in a counterclockwise rotation/left turn
        if (((360 - angle) + startingAngle) % 360 > 180) {
            while (((360 - angle) + gps.getRot()) % 360 > 180) {
                if (!isStopRequested() && opModeIsActive()) {
                    this.turnLeft(power);
                } else {
                    break;
                }
            }
        } else {
            // while the left turn angle value is less than or equal to 180, turn left
            while (((360 - angle) + gps.getRot()) % 360 <= 180) {
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
     * Turns to the provided angle
     *
     * @param angle         The angle to turn to in degrees
     * @param maxPower      The maximum power the robot will turn at
     * @param minPower      The minimum power the robot will turn at
     * @param consoleOutput If this is true, it will print extra information via telemetry
     * @throws InterruptedException Throws if the OpMode is killed
     */
    public void newTurnToPrototype(double angle, double maxPower, double minPower, boolean consoleOutput) throws InterruptedException {

        angle = angle % 360;
        //the following calculation determines the value of the angle between the current startingAngle and the desired endingAngle in a clockwise rotation/right turn
        double initialDegreesOff = ((360 - angle) + gps.getRot()) % 360;
        double degreesOff = ((360 - angle) + gps.getRot()) % 360;
        double pow;
        final double maximumPower = maxPower; //The minimum power the robot can turn at
        final double minimumPower = minPower; //The minimum power the robot can turn at


        if (initialDegreesOff < 180) {
            while (degreesOff <= 180 && !isStopRequested() && opModeIsActive()) {
                // this is the degreesOff in a right turning motion
                degreesOff = ((360 - angle) + gps.getRot()) % 360;
                pow = shortMovementPowerCalculation(initialDegreesOff, degreesOff, maximumPower, minimumPower);

                this.turnRight(MiscUtils.boundNumber(pow));
                //Thread.sleep(20); //A sleep because we do not want this loop running at full speed. We don't need it to.
                if (consoleOutput) {
                    telemetry.addData("rotation", gps.getRot());
                    telemetry.addData("power", pow);
                    telemetry.update();
                }
            }

        } else {
            //
            degreesOff = (angle + (360 - gps.getRot())) % 360;
            // this is the degreesOff in a left turning motion
            while (degreesOff <= 180 && !isStopRequested() && opModeIsActive()) {

                degreesOff = (angle + (360 - gps.getRot())) % 360;
                pow = shortMovementPowerCalculation(initialDegreesOff, degreesOff, maximumPower, minimumPower);


                this.turnLeft(MiscUtils.boundNumber(pow));
                //Thread.sleep(20); //A sleep because we do not want this loop running at full speed. We don't need it to.
                if (consoleOutput) {
                    telemetry.addData("rotation", gps.getRot());
                    telemetry.addData("power", pow);
                    telemetry.update();
                }
            }
        }
        stopAll();
    }

    /**
     * this calculates the ideal power for a movement based on the robots position in a short movement.
     *
     * @param maximumPower    the maximum power output of the robot during the movement
     * @param minimumPower    the minimum power output of the robot during the movement
     * @param currentDistance this is the distance from the current position of the movement to the desired position of the movement
     *                        when this is near current distance, it is near maximum power
     *                        when this reaches 0, it is near minimum power
     * @param initialDistance this is the desired position of the movement as a double
     * @return this returns the power value of the movement with the given parameters
     */
    public double shortMovementPowerCalculation(double initialDistance, double currentDistance, double maximumPower, double minimumPower) {
        // currently this is only being used in turning related functions
        double powerOutput = MiscUtils.boundNumber(Math.sin((Math.PI * currentDistance) / (2 * initialDistance)));
        return MiscUtils.map(powerOutput, 0, 1, minimumPower, maximumPower);
    }

    //-Move To Position Methods----------------------------------------------------------------------------------------------

    /**
     * Moves the robot to the provided position Enum. Upon the end of the function call, the robot is stopped
     *
     * @param position  a hashmap value referencing the 2 value array of the position
     * @param tolerance The distance the robot can be off from the given position
     * @throws InterruptedException Throws if the opMode is stopped
     */
    public void moveToPosition(FieldPoints position, double tolerance) throws InterruptedException {
        double[] pos = FieldPoints.positionsAndPoints.get(position);
        assert pos != null;
        moveToPosition(pos[0], pos[1], gps.getRot(), tolerance);
        this.stopAll();
    }

    /**
     * Moves the robot to the provided position Enum. Upon the end of the function call, the robot is stopped
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
     * Moves the robot to the given (x,y) position. Upon the end of the function call, the robot is stopped
     *
     * @param x         X Value to move to
     * @param y         Y Value to move to
     * @param tolerance The distance the robot can from the given position (in inches)
     * @throws InterruptedException Throws an exception if stop is requested during the move
     */
    public void moveToPosition(double x, double y, double tolerance) throws InterruptedException {
        moveToPosition(x, y, gps.getRot(), tolerance);
    }

    /**
     * Moves the robot to the given (x,y) position and turns to the angle (theta). Upon the end of the function call, the robot is stopped
     *
     * @param x         The x-coordinate to move to
     * @param y         The y-coordinate to move to
     * @param theta     The angle to turn to, relative to the field
     * @param tolerance The tolerance for how close it must get (in inches)
     * @throws InterruptedException Throws if stop is requested during this time
     */
    public void moveToPosition(double x, double y, double theta, double tolerance) throws InterruptedException {
        try {
            moveToPosition(x, y, theta, tolerance, new MovementException());
        } catch (MovementException ignored) {
        }
    }

    /**
     * Moves the robot to the given (x,y) position and turns to the given angle (theta). Stops robot and throws according to the error conditions provided
     *
     * @param x         The x coordinate to go to
     * @param y         The y coordinate to go to
     * @param tolerance The tolerance for how close it must get (in inches)
     * @param theta     The angle (relative to the field) to turn to during the movement
     * @param errors    A array of error conditions to check. Must be of type {@link MovementException}
     * @throws InterruptedException Throws if the OpMode ends during execution
     * @throws MovementException    Stops Motors and Throws if errorCB returns true
     */
    public void moveToPosition(double x, double y, double theta, double tolerance, MovementException[] errors) throws InterruptedException, MovementException {
        double power;

        final String args = "moveToPosition(" + x + ", " + y + ", " + theta + ", " + tolerance + ")\n";
        final String coordinateString = x + " , " + y;

        double[] currentPos = gps.getPos();
        double currentX = currentPos[0];
        double currentY = currentPos[1];
        double currentAngle = currentPos[2];

        double currentDistance = MiscUtils.distance(currentX, currentY, x, y);

        while (currentDistance > tolerance && !isStopRequested() && opModeIsActive()) {

            /*The next if-else block takes the distance from target and
             sets the power variable to currentAngle power following the function
             @param zeroPoint is the point where the robot goes at 1
             power = 0.8/zeroPoint(distance) + 0.2
             if the distance is greater than 24 in or ~2 ft, robot moves at power of 1
             */
            final double zeroPoint = 24;
            if (currentDistance > zeroPoint) {
                power = 1;
            } else {
                //power = ((0.9 / zeroPoint) * currentDistance) + 0.2;
                power = shortMovementPowerCalculation(zeroPoint, currentDistance, .9, .075);
                // this power function has yet to be tested
            }
            if (this.debug) {
                telemetry.addData("Function", args);
                telemetry.addData("Moving to", coordinateString);
                telemetry.addData("currentDistance", currentDistance);
                telemetry.addData("angle", currentAngle);
                telemetry.addData("Moving?", (currentDistance > tolerance && !isStopRequested() && opModeIsActive()));
                telemetry.addData("X Pos", currentX);
                telemetry.addData("Y Pos", currentY);
                telemetry.addData("Power", power);
                telemetry.update();
            }

            currentPos = gps.getPos();
            currentX = currentPos[0];
            currentY = currentPos[1];
            currentAngle = MiscUtils.getAngle(currentX, currentY, x, y, currentPos[2]); //angle;

            currentDistance = MiscUtils.distance(currentX, currentY, x, y); //currentDistance value


            for (MovementException e : errors) {
                try {
                    e.call(x, y, theta, tolerance, telemetry, gps, _isStopRequested, _opModeIsActive, voltageSensor);
                } catch (MovementWarning ignored) {
                    this.stopAll();
                    return;
                } catch (MovementException Me) {
                    this.stopAll();
                    throw Me;
                }
            }

            if (turnWhileStrafe) {
                strafeAtAngleWhileTurn(currentAngle, theta, power);
            } else {
                strafeAtAngle(currentAngle, power);
            }


        }
        stopAll();
        telemetry.clear();
        telemetry.update();
    }

    /**
     * Moves the robot to the given (x,y) position and turns to the given angle (theta). Stops robot and returns according to the warning supplied
     *
     * @param x         The x coordinate to go to
     * @param y         The y coordinate to go to
     * @param tolerance The tolerance for how close it must get (in inches)
     * @param theta     The angle (relative to the field) to turn to during the movement
     * @param warning   A {@link MovementWarning} object. If it throws, the robot stops and this function returns
     * @throws InterruptedException Throws if the OpMode ends during execution
     */
    public void moveToPosition(double x, double y, double theta, double tolerance, MovementWarning warning) throws InterruptedException {
        MovementWarning[] errors = {warning};
        moveToPosition(x, y, theta, tolerance, errors);

    }

    /**
     * Moves the robot to the given (x,y) position and turns to the given angle (theta). Stops robot and returns according to the warning supplied
     *
     * @param x         The x coordinate to go to
     * @param y         The y coordinate to go to
     * @param tolerance The tolerance for how close it must get (in inches)
     * @param theta     The angle (relative to the field) to turn to during the movement
     * @param warnings  A array of {@link MovementWarning} objects. If one of them throws, the robot stops and this function returns
     * @throws InterruptedException Throws if the OpMode ends during execution
     */
    public void moveToPosition(double x, double y, double theta, double tolerance, MovementWarning[] warnings) throws InterruptedException {
        try {
            moveToPosition(x, y, theta, tolerance, (MovementException[]) warnings);
        } catch (MovementException ignored) {
            this.stopAll();
        }
    }

    /**
     * Moves the robot to the given (x,y) position and turns to the given angle (theta). Stops robot and returns according to the warning supplied
     *
     * @param x         The x coordinate to go to
     * @param y         The y coordinate to go to
     * @param tolerance The tolerance for how close it must get (in inches)
     * @param theta     The angle (relative to the field) to turn to during the movement
     * @param error     An ={@link MovementException} objects. If it, the robot stops and the exception is sent up the call stack
     * @throws InterruptedException Throws if the OpMode ends during execution
     * @throws MovementException    Throws if the error condition throws
     */
    public void moveToPosition(double x, double y, double theta, double tolerance, MovementException error) throws MovementException, InterruptedException {
        MovementException[] errors = {error};
        moveToPosition(x, y, theta, tolerance, errors);
    }

    //-Move Towards Position Methods----------------------------------------------------------------------------------------------

    /**
     * Moves towards the desired location. Upon the end of the function call, the robot is not stopped
     *
     * @param x         The x coordinate to move towards
     * @param y         The y coordinate to move towards
     * @param theta     The angle to turn to
     * @param power     The power to move at
     * @param tolerance The tolerance for how close the robot must get
     * @param warning   A {@link MovementWarning} object. If it throws, the exception is swallowed, the robot does not stop and this function returns
     * @throws InterruptedException Throws if the OpMode ends during execution
     */
    public void moveTowardsPosition(double x, double y, double theta, final double power, double tolerance, MovementWarning warning) throws InterruptedException {
        try {
            this.moveTowardsPosition(x, y, theta, power, tolerance, new MovementException[]{warning});
        } catch (MovementException ignored) {
        }
    }

    /**
     * Moves towards the desired location. Upon the end of the function call, the robot is not stopped
     *
     * @param x         The x coordinate to move towards
     * @param y         The y coordinate to move towards
     * @param theta     The angle to turn to
     * @param power     The power to move at
     * @param tolerance The tolerance for how close the robot must get
     * @param warnings  A array of {@link MovementWarning} objects. If it throws, the exception is swallowed, the robot does not stop and this function returns
     * @throws InterruptedException Throws if the OpMode ends during execution
     */
    public void moveTowardsPosition(double x, double y, double theta, final double power, double tolerance, MovementWarning[] warnings) throws InterruptedException {
        try {
            this.moveTowardsPosition(x, y, theta, power, tolerance, (MovementException[]) warnings);
        } catch (MovementException ignored) {
        }
    }

    /**
     * Moves towards the desired location. Upon the end of the function call, the robot is not stopped. Throws if error condition is met
     *
     * @param x         The x coordinate to move towards
     * @param y         The y coordinate to move towards
     * @param theta     The angle to turn to
     * @param power     The power to move at
     * @param tolerance The tolerance for how close the robot must get
     * @param error     An {@link MovementException} object. If it throws, the robot does not stop and the exception is propagated up the stack
     * @throws InterruptedException Throws if the OpMode ends during execution
     * @throws MovementException    Throws if error throws
     */
    public void moveTowardsPosition(double x, double y, double theta, final double power, double tolerance, MovementException error) throws InterruptedException, MovementException {
        this.moveTowardsPosition(x, y, theta, power, tolerance, new MovementException[]{error});
    }

    /**
     * Moves towards the desired location. Upon the end of the function call, the robot is not stopped. Throws if an error condition is met
     *
     * @param x         The x coordinate to move towards
     * @param y         The y coordinate to move towards
     * @param theta     The angle to turn to
     * @param power     The power to move at
     * @param tolerance The tolerance for how close the robot must get
     * @param errors    A array of error conditions to check. If it throws, the robot does not stop and the exception is propagated up the stack
     * @throws InterruptedException Throws if the OpMode ends during execution
     * @throws MovementException    Throws if error throws
     */
    public void moveTowardsPosition(double x, double y, double theta, final double power, double tolerance, MovementException[] errors) throws InterruptedException, MovementException {

        final String args = "moveTowardsPosition(" + x + ", " + y + ", " + theta + ", " + power + ")\n";
        final String coordinateString = x + " , " + y;

        double[] currentPos = gps.getPos();
        double currentX = currentPos[0];
        double currentY = currentPos[1];
        double currentAngle = currentPos[2];

        double currentDistance = MiscUtils.distance(currentX, currentY, x, y);

        while (currentDistance > tolerance && !isStopRequested() && opModeIsActive()) {

            if (this.debug) {
                telemetry.addData("Function", args);
                telemetry.addData("Moving to", coordinateString);
                telemetry.addData("currentDistance", currentDistance);
                telemetry.addData("angle", currentAngle);
                telemetry.addData("Moving?", (currentDistance > tolerance && !isStopRequested() && opModeIsActive()));
                telemetry.addData("X Pos", currentX);
                telemetry.addData("Y Pos", currentY);
                telemetry.addData("Power", power);
                telemetry.update();
            }

            currentPos = gps.getPos();
            currentX = currentPos[0];
            currentY = currentPos[1];
            currentAngle = MiscUtils.getAngle(currentX, currentY, x, y, currentPos[2]); //angle;

            currentDistance = MiscUtils.distance(currentX, currentY, x, y); //currentDistance value


            for (MovementException e : errors) {
                e.call(x, y, theta, tolerance, telemetry, gps, _isStopRequested, _opModeIsActive, voltageSensor);
            }

            if (turnWhileStrafe) {
                strafeAtAngleWhileTurn(currentAngle, theta, power);
            } else {
                strafeAtAngle(currentAngle, power);
            }
        }
    }

    //-Misc Methods----------------------------------------------------------------------------------------------


    /**
     * Strafes at the provided angle relative to the robot, and turns to the given turnAngle
     *
     * @param angle     The angle to strafe at
     * @param turnAngle The angle to turn to
     * @param power     The power to turn at
     */
    public void strafeAtAngleWhileTurn(double angle, double turnAngle, double power) {

        power = MiscUtils.boundNumber(power);
        double power1;
        double power2;
        double power3;
        double power4;

        angle = angle % 360;
        //----strafing power calculation---------------------------------
        power1 = -Math.cos(Math.toRadians(angle + 45.0)); //power 1 is front right and back left
        power2 = -Math.cos(Math.toRadians(angle - 45)); // power 2 is front right and back left

        power1 = power * power1;
        power2 = power * power2;

        //-----turning power calculation-------------------------------------
        double degreesOff = ((gps.getRot() - turnAngle) % 360);
        double tmp;
        if (degreesOff < 180) {
            tmp = MiscUtils.map(degreesOff, 0, 180, 0, 1);
        } else {
            tmp = MiscUtils.map(degreesOff, 180, 360, -1, 0);
        }
        power3 = -tmp;
        power4 = tmp;

        //---setting motor powers--------------------------------------------
        front_right.setPower(power1 + power3);
        back_left.setPower(power1 + power4);

        front_left.setPower(power2 + power4);
        back_right.setPower(power2 + power3);

    }

    /**
     * Strafes at the provided angle relative to the robot, and turns to the given turnAngle
     *
     * @param angle     The angle to strafe at
     * @param turnAngle The angle to turn to
     * @param power     The power to turn at
     */
    private void newStrafeAtAngleWhileTurnPrototype(double angle, double turnAngle, double power) {

        final boolean factorInAngleTurn = false;
        power = MiscUtils.boundNumber(power);
        double power1;
        double power2;
        double power3;
        double power4;

        angle = angle % 360;

        //----strafing power calculation---------------------------------
        power1 = -Math.cos(Math.toRadians(angle + 45.0)); //power 1 is front right and back left
        power2 = -Math.cos(Math.toRadians(angle - 45)); // power 2 is front right and back left

        power1 = power * power1;
        power2 = power * power2;

        //-----turning power calculation-------------------------------------
        double degreesOffClockwise = ((360 - turnAngle) + (gps.getRot())) % 360;
        double tmp;
        if (degreesOffClockwise < 180) {
            tmp = MiscUtils.map(degreesOffClockwise, 0, 180, 0, 1);
        } else {
            tmp = MiscUtils.map(degreesOffClockwise, 180, 360, -1, 0);
        }
        power3 = -tmp;
        power4 = tmp;

        //---setting motor powers--------------------------------------------
        front_right.setPower(power1 + power3);
        back_left.setPower(power1 + power4);

        front_left.setPower(power2 + power4);
        back_right.setPower(power2 + power3);

    }

    //move Methods -------------------------------------------------------------------------------------------

    public void move(double angle, double distance, double tolerance) throws InterruptedException {
        angle += gps.getRot();
        angle = Math.toRadians(angle);
        double x = gps.getX() + distance * Math.sin(angle);
        double y = gps.getY() + distance * Math.cos(angle);
        moveToPosition(x, y, tolerance);
    }

    public void move(double angle, double distance, double tolerance, MovementException[] errors) throws InterruptedException, MovementException {
        angle += gps.getRot();
        angle = Math.toRadians(angle);
        double x = gps.getX() + distance * Math.sin(angle);
        double y = gps.getY() + distance * Math.cos(angle);
        moveToPosition(x, y, gps.getRot(), tolerance, errors);
    }

    public void move(double angle, double distance, double tolerance, MovementWarning[] warnings) throws InterruptedException {
        try {
            move(angle, distance, tolerance, (MovementException[]) warnings);
        } catch (MovementException ignored) {
        }
    }

    public void move(double angle, double distance, double tolerance, MovementException exception) throws MovementException, InterruptedException {
        move(angle, distance, tolerance, new MovementException[]{exception});
    }

    public void move(double angle, double distance, double tolerance, MovementWarning warning) throws InterruptedException {
        try {
            move(angle, distance, tolerance, new MovementException[]{warning});
        } catch (MovementException ignored) {
        }
    }


}
