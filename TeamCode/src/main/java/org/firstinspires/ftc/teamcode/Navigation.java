package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Objects;


/** Keeps track of the robot's desired path and makes it follow it accurately.
 */
public class Navigation {
    public enum RotationDirection {CLOCKWISE, COUNTERCLOCKWISE}
    public static enum Action {NONE, DELIVER_CONE_LOW, DELIVER_CONE_MEDIUM, DELIVER_CONE_HIGH, DELIVER_CONE_HIGH_90,
        PICK_UP_FIRST_STACK_CONE, PICK_UP_SECOND_STACK_CONE}
    // AUTON CONSTANTS
    // ===============
    public enum MovementMode {FORWARD_ONLY, STRAFE}
    static final double STRAFE_ACCELERATION = 0.1; // Inches per second squared
    static final double ROTATE_ACCELERATION = 0.1; // Radians per second squared
    static final double SPEED_FACTOR = 1.0; // Speed of robot when all motors are set to full power
    // TODO: empirically measure speed factor
    static final double ROTATION_RADIUS = 1.0; //Radius of the robot's rotation
    // TODO: empirically measure and calculate rotation radius
    static final double STRAFE_RAMP_DISTANCE = 3;  // Inches
    static final double ROTATION_RAMP_DISTANCE = Math.PI / 3;  // Radians
    static final double MIN_STRAFE_POWER = 0.3;
    static final double MAX_STRAFE_POWER = 0.5;
    static final double STRAFE_CORRECTION_POWER = 0.3;
    static final double MAX_ROTATION_POWER = 0.5;
    static final double MIN_ROTATION_POWER = 0.4;
    static final double ROTATION_CORRECTION_POWER = 0.2;
    // Accepted amounts of deviation between the robot's desired position and actual position.
//    static final double EPSILON_LOC = 1.4;
    static final double EPSILON_LOC = 10;

    static final double EPSILON_ANGLE = 0.35;
    // The number of frames to wait after a rotate or travelLinear call in order to check for movement from momentum.
    static final int NUM_CHECK_FRAMES = 10;

    // Distance between starting locations on the warehouse side and the carousel side.
    static final double DISTANCE_BETWEEN_START_POINTS = 35.25;
    static final double RED_BARCODE_OFFSET = 0;

    // Distances between where the robot extends/retracts the linear slides and where it opens the claw.
    static final double HORSESHOE_SIZE = 8.9;

    static final double FLOAT_EPSILON = 0.001;

    public MovementMode movementMode;

    // TELEOP CONSTANTS
    // ================
    static final double MOVEMENT_MAX_POWER = 1;
    static final double ROTATION_POWER = 0.5;
    static final double SLOW_MOVEMENT_SCALE_FACTOR = 0.3;
    static final double MEDIUM_MOVEMENT_SCALE_FACTOR = 0.6;

    // INSTANCE ATTRIBUTES
    // ===================

    // Speeds relative to one another. RR is positive on purpose!
    //                              RL   RR   FL   FR
    public double[] wheel_speeds = {1.0, 1.0, -0.97, -1.0};
//    public double[] wheel_speeds = {0.3, 0.3, -0.291, -0.3};

    public double strafePower;  // Tele-Op only

    // First position in this ArrayList is the first position that robot is planning to go to.
    // This condition must be maintained (positions should be deleted as the robot travels)
    // NOTE: a position is both a location and a rotation.
    // NOTE: this can be changed to a stack later if appropriate (not necessary for speed, just correctness).
    public ArrayList<Position> path;
    public int pathIndex;

    public Navigation(ArrayList<Position> path, RobotManager.AllianceColor allianceColor,
                      RobotManager.StartingSide startingSide, MovementMode movementMode) {
        this.path = path;
        this.movementMode = movementMode;
        pathIndex = 0;
    }
    public void configurePath(RobotManager.StartingSide startingSide, RobotManager.ParkingPosition parking_position) {
        transformPath(startingSide);
        //Set parking location
        setParkingLocation(startingSide, parking_position);

    }

    /** Makes the robot travel along the path until it reaches a POI.
     */
    public Position travelToNextPOI(Robot robot) {
        while (true) {
            if (path.size() <= pathIndex) {
                robot.telemetry.addData("Path size <= to path index, end of travel. pathIndex:", pathIndex);
                return null;
            }
            Position target = path.get(pathIndex);
            robot.positionManager.updatePosition(robot);
            robot.telemetry.addData("Going to", target.getX() + ", " + target.getY());
            robot.telemetry.addData("name", target.getName());
            robot.telemetry.update();
            switch (movementMode) {
                case FORWARD_ONLY:
                    rotate(getAngleBetween(robot.getPosition(), target) - Math.PI / 2,
                            target.rotatePower, robot);
                    travelLinear(target, target.getStrafePower(), robot);
                    rotate(target.getRotation(), target.getRotatePower(), robot);
                    break;
                case STRAFE:
                    travelLinear(target, target.strafePower, robot);
                    rotate(target.getRotation(), target.rotatePower, robot);
                    break;
            }

            pathIndex++;

            robot.telemetry.addData("Got to", target.name);
            robot.telemetry.update();

//            if (target.name.startsWith("POI")) break;
        }
//        return path.get(pathIndex - 1);
    }

    /** Updates the strafe power according to movement mode and gamepad 1 left trigger.
     */
    public void updateStrafePower(boolean hasMovementDirection, GamepadWrapper gamepads, Robot robot) {
        if (!hasMovementDirection) {
            strafePower = 0;
            return;
        }

        AnalogValues analogValues = gamepads.getAnalogValues();

        double distance = Range.clip(Math.sqrt(Math.pow(analogValues.gamepad1LeftStickX, 2)
                + Math.pow(analogValues.gamepad1LeftStickY, 2)), 0, 1);
        if (distance <= RobotManager.JOYSTICK_DEAD_ZONE_SIZE) {  // joystick dead zone
            // Joystick is not used, but hasMovementDirection is true, so one of the straight movement buttons must
            // have been pressed.
            strafePower = SLOW_MOVEMENT_SCALE_FACTOR;
        } else {
            strafePower = distance * MOVEMENT_MAX_POWER;
            if (Robot.desiredSlidesState == Robot.SlidesState.HIGH && robot.slidesMotor1.getPower() == 0) {
                strafePower *= SLOW_MOVEMENT_SCALE_FACTOR;
            } else if (Robot.desiredSlidesState == Robot.SlidesState.MEDIUM && robot.slidesMotor1.getPower() == 0) {
                strafePower *= MEDIUM_MOVEMENT_SCALE_FACTOR;
            } else if (Robot.desiredSlidesState == Robot.SlidesState.LOW && robot.slidesMotor1.getPower() == 0) {
                strafePower *= MEDIUM_MOVEMENT_SCALE_FACTOR; //Scale factor is the same as medium
            }
        }
    }

    /** Moves the robot straight in one of the cardinal directions or at a 45 degree angle.
     *
     *  @return whether any of the D-Pad buttons were pressed.
     */
    public boolean moveStraight(boolean forward, boolean backward, boolean left, boolean right, Robot robot) {
        double direction;
        if(forward || backward) {
            if (left) {//move NW
                direction = Math.PI * 0.75;
            }
            else if (right) {//move NE
                direction = Math.PI * 0.25;
            }
            else {//move just forward
                direction = Math.PI * 0.5;
            }
            if (backward) {
                direction *= -1;
            }
        }
        else if (left) {
            direction = Math.PI;
        }
        else if (right) {
            direction = 0.0;
        }
        else {
            return false;
        }
        setDriveMotorPowers(direction, strafePower, 0.0, robot, false);
        return true;
    }

    /** Changes drivetrain motor inputs based off the controller inputs.
     */
    public void maneuver(AnalogValues analogValues, Robot robot) {
        // Uses left stick to go forward, and right stick to turn.
        // NOTE: right-side drivetrain motor inputs don't have to be negated because their directions will be reversed
        //       upon initialization.

        double turn = -analogValues.gamepad1RightStickX;
        if (Math.abs(turn) < RobotManager.JOYSTICK_DEAD_ZONE_SIZE) {
            turn = 0;
        }

        double moveDirection = Math.atan2(analogValues.gamepad1LeftStickY, analogValues.gamepad1LeftStickX);
        if (Math.abs(moveDirection) < Math.PI / 12) {
            moveDirection = 0.0;
        }
        else if (Math.abs(moveDirection - Math.PI / 2) < Math.PI / 12) {
            moveDirection = Math.PI / 2;
        }
        else if (Math.abs(moveDirection - Math.PI) % Math.PI < Math.PI / 12) {
            moveDirection = Math.PI;
        }
        else if (Math.abs(moveDirection + Math.PI / 2) < Math.PI / 12) {
            moveDirection = -Math.PI / 2;
        }

        // Field-centric navigation
//        moveDirection -= robot.positionManager.position.getRotation();

        setDriveMotorPowers(moveDirection, strafePower, turn * ROTATION_POWER, robot, false);
    }

    /** Rotates the robot a number of degrees.
     *
     *  @param target The orientation the robot should assume once this method exits.
     *               Within the interval (-pi, pi].
     *  @param constantPower A hard-coded power value for the method to use instead of ramping. Ignored if set to zero.
     */
    public void rotate(double target, double constantPower, Robot robot) {
        robot.positionManager.updatePosition(robot);
        // Both values are restricted to interval (-pi, pi].
        final double startOrientation = robot.getPosition().getRotation();
        double currentOrientation = startOrientation;  // Copies by value because double is primitive.
        double startingTime = robot.elapsedTime.milliseconds();

        double rotationSize = getRotationSize(startOrientation, target);
        double halfRotateTime = getHalfRotateTime(rotationSize);

        double power;
        boolean ramping = true;
        if (Math.abs(constantPower - 0) > FLOAT_EPSILON) {
            power = constantPower;
            ramping = false;
        }
        else {
            power = MIN_ROTATION_POWER;
        }

        double rotationProgress = getRotationSize(startOrientation, currentOrientation);
        double rotationRemaining = getRotationSize(currentOrientation, target);
        boolean finishedRotation = false;
        int numFramesSinceLastFailure = 0;
        boolean checkFrames = false;
        double timeElapsed = 0;
        double timeRemaining = 2 * halfRotateTime - timeElapsed;

        while (rotationRemaining > EPSILON_ANGLE) {
            robot.telemetry.addData("rot left", rotationRemaining);
            robot.telemetry.addData("current orientation", currentOrientation);
            robot.telemetry.addData("target", target);
            robot.telemetry.update();

            if (ramping) {
                if (timeElapsed < halfRotateTime) { // Ramping up
                    power = Range.clip(
                            (timeElapsed / halfRotateTime) * MAX_ROTATION_POWER,
                            MIN_ROTATION_POWER, MAX_ROTATION_POWER);
                }
                else { // Ramping down
                    power = Range.clip(
                            (timeRemaining / halfRotateTime) * MAX_ROTATION_POWER,
                            MIN_ROTATION_POWER, MAX_ROTATION_POWER);
                }
            }

            if (checkFrames) {
                power = ROTATION_CORRECTION_POWER;
            }

            switch (getRotationDirection(currentOrientation, target)) {
                case CLOCKWISE:
                    setDriveMotorPowers(0.0, 0.0, power, robot, false);
                    break;
                case COUNTERCLOCKWISE:
                    setDriveMotorPowers(0.0, 0.0, -power, robot, false);
                    break;
            }

            robot.positionManager.updatePosition(robot);
            currentOrientation = robot.getPosition().getRotation();

            rotationProgress = getRotationSize(startOrientation, currentOrientation);
            rotationRemaining = getRotationSize(currentOrientation, target);
            timeElapsed = robot.elapsedTime.milliseconds() - startingTime;
            timeRemaining = 2 * halfRotateTime - timeElapsed;

            if (rotationRemaining > EPSILON_ANGLE) {
                numFramesSinceLastFailure = 0;
            } else {
                checkFrames = true;
                numFramesSinceLastFailure++;
                if (numFramesSinceLastFailure >= NUM_CHECK_FRAMES) {
                    finishedRotation = true;
                }
            }
        }

        stopMovement(robot);
    }

    /** Determines whether the robot has to turn clockwise or counterclockwise to get from theta to target.
     *
     *  @param theta the current orientation
     *  @param target the desired orientation
     */
    private RotationDirection getRotationDirection(double theta, double target) {
        double angleDiff = target - theta;  // Counterclockwise distance to target
        if ((angleDiff >= -Math.PI && angleDiff < 0) || (angleDiff > Math.PI)) {
            return RotationDirection.CLOCKWISE;
        }
        return RotationDirection.COUNTERCLOCKWISE;
    }

    /** Calculates the number of radians of rotation required to get from theta to target.
     *
     *  @param theta the current orientation
     *  @param target the desired orientation
     */
    private double getRotationSize(double theta, double target) {
        double rotationSize = Math.abs(target - theta);
        if (rotationSize > Math.PI) {
            rotationSize = 2 * Math.PI - rotationSize;
        }
        return rotationSize;
    }

    /** Makes the robot travel in a straight line for a certain distance.
     *
     *  @param target The desired position of the robot.
     *  @param constantPower A hard-coded power value for the method to use instead of ramping. Ignored if set to zero.
     */
    public void travelLinear(Position target, double constantPower, Robot robot) {
        robot.positionManager.updatePosition(robot);
        final Position startPosition = robot.getPosition();
        final double startX = startPosition.getX();
        final double startY = startPosition.getY();
        Position currentPosition = robot.getPosition();
        double currentX = currentPosition.getX();
        double currentY = currentPosition.getY();
        double startingTime = robot.elapsedTime.milliseconds();

        double totalDistance = getEuclideanDistance(startPosition, target);
        double halfStrafeTime = getHalfStrafeTime(totalDistance, getAngleBetween(startPosition, target));

        double power;
        boolean ramping = true;

        if (Math.abs(constantPower - 0.0) > FLOAT_EPSILON) {
            power = constantPower;
            ramping = false;
        }
        else {
            power = MIN_STRAFE_POWER;
        }

        double distanceTraveled = getEuclideanDistance(startPosition, currentPosition);
        double distanceRemaining = getEuclideanDistance(currentPosition, target);
        boolean finishedTravel = false;
        double numFramesSinceLastFailure = 0;
        boolean checkFrames = false;
        double timeElapsed = 0;
        double timeRemaining = 2 * halfStrafeTime - timeElapsed;

        while (distanceRemaining > EPSILON_LOC) {

            robot.positionManager.updatePosition(robot);

            if (ramping) {
                if (timeElapsed < halfStrafeTime) { // Ramping up
                    power = Range.clip(
                            (timeElapsed / halfStrafeTime) * MAX_STRAFE_POWER,
                            MIN_STRAFE_POWER, MAX_STRAFE_POWER);
                }
                else { // Ramping down
                    power = Range.clip(
                            (timeRemaining / halfStrafeTime) * MAX_STRAFE_POWER,
                            MIN_STRAFE_POWER, MAX_STRAFE_POWER);
                }
            }

            if (checkFrames) {
                power = STRAFE_CORRECTION_POWER;
            }

            double strafeAngle = getStrafeAngle(robot.getPosition(), target);
            robot.telemetry.addData("starting rotation", robot.getPosition().getRotation());
            robot.telemetry.addData("used strafe angle", strafeAngle);

            setDriveMotorPowers(strafeAngle, power, 0.0, robot, false);

            robot.positionManager.updatePosition(robot);
            currentPosition = robot.getPosition();
            distanceTraveled = getEuclideanDistance(startPosition, currentPosition);
            distanceRemaining = getEuclideanDistance(currentPosition, target);
            timeElapsed = robot.elapsedTime.milliseconds() - startingTime;
            timeRemaining = 2 * halfStrafeTime - timeElapsed;

            robot.telemetry.addData("Start X", startX);
            robot.telemetry.addData("Start Y", startY);
            robot.telemetry.addData("Current X", currentX);
            robot.telemetry.addData("Current Y", currentY);

            robot.telemetry.addData("Target X", target.x);
            robot.telemetry.addData("Target Y", target.y);
            robot.telemetry.addData("strafe testing y", (target.y-currentY));
            robot.telemetry.addData("strafe testing x", (target.x-currentX));
            robot.telemetry.addData("get angle between angle", getAngleBetween(currentPosition, target));
            robot.telemetry.update();

            if (distanceRemaining > EPSILON_LOC) {
                numFramesSinceLastFailure = 0;
            }
            else {
                checkFrames = true;
                numFramesSinceLastFailure++;
                if (numFramesSinceLastFailure >= NUM_CHECK_FRAMES) {
                    finishedTravel = true;
                }
            }
        }
        stopMovement(robot);
    }

    /** Calculates the angle at which the robot must strafe in order to get to a target location.
     *
     *  @param currentLoc the current position
     *  @param target the desired position
     */
    private double getStrafeAngle(Position currentLoc, Position target) {

        double strafeAngle = getAngleBetween(currentLoc, target) - currentLoc.getRotation();
        if (strafeAngle > Math.PI) {
            strafeAngle -= 2 * Math.PI;
        }
        else if (strafeAngle < -Math.PI) {
            strafeAngle += 2 * Math.PI;
        }
//        return 0;
        return strafeAngle;
    }

    /** Calculates the speed of the robot when strafing given the direction of strafing and the strafing speed
     *
     *  @param strafeAngle the direction (angle) in which the robot should strafe
     *  @param power the strafing speed of the robot
     *  @return the speed of the robot
     */
    private double getRobotSpeed(double strafeAngle, double power) {
        double powerSet1 = Math.sin(strafeAngle) + Math.cos(strafeAngle);
        double powerSet2 = Math.sin(strafeAngle) - Math.cos(strafeAngle);
        double[] rawPowers = scaleRange(powerSet1, powerSet2);

        if (strafeAngle > -Math.PI/2 && strafeAngle < Math.PI/2) {
            return (SPEED_FACTOR * power * Math.sqrt(2 * Math.pow(rawPowers[1]/rawPowers[0], 2) + 2)) / 2;
        }
        else {
            return (SPEED_FACTOR * power * Math.sqrt(2 * Math.pow(rawPowers[0]/rawPowers[1], 2) + 2)) / 2;
        }
    }

    /** Calculates half the amount of time it is estimated for a linear strafe to take.
     *
     *  @param distance the distance of the strafe
     *  @param strafeAngle the angle of the strafe
     */
    private double getHalfStrafeTime(double distance, double strafeAngle) {
        // Inches per second
        double min_strafe_speed = getRobotSpeed(strafeAngle, MIN_STRAFE_POWER);
        double max_strafe_speed = getRobotSpeed(strafeAngle, MAX_STRAFE_POWER);


        double ramp_distance = (max_strafe_speed + min_strafe_speed) / 2
                * (max_strafe_speed - min_strafe_speed) / STRAFE_ACCELERATION;
        if (distance / 2 >= ramp_distance) {
            return (distance / 2 - ramp_distance) / max_strafe_speed
                    + (max_strafe_speed - min_strafe_speed) / STRAFE_ACCELERATION;
        }
        else {  // We never get to max_strafe_speed
            return (-min_strafe_speed + Math.sqrt(Math.pow(min_strafe_speed, 2) + distance * STRAFE_ACCELERATION))
                    / 0.5 * STRAFE_ACCELERATION;
        }
    }

    /** Calculates half the amount of time it is estimated for a rotation to take.
     *
     *  @param angle The angle of the rotation
     */
    private double getHalfRotateTime(double angle) {
        // Radians per second
        double min_rotate_speed = (SPEED_FACTOR * MIN_ROTATION_POWER) / ROTATION_RADIUS;
        double max_rotate_speed = (SPEED_FACTOR * MAX_ROTATION_POWER) / ROTATION_RADIUS;


        double ramp_angle = (max_rotate_speed + min_rotate_speed) / 2
                * (max_rotate_speed - min_rotate_speed) / ROTATE_ACCELERATION;
        if (angle / 2 >= ramp_angle) {
            return (angle / 2 - ramp_angle) / max_rotate_speed
                    + (max_rotate_speed - min_rotate_speed) / ROTATE_ACCELERATION;
        }
        else {  // We never get to max_rotate_speed
            return (-min_rotate_speed + Math.sqrt(Math.pow(min_rotate_speed, 2) + angle * ROTATE_ACCELERATION))
                    / 0.5 * ROTATE_ACCELERATION;
        }
    }

    /** Determines the angle between the horizontal axis (currently left side of the robot) and the segment connecting A and B.
     *  Currently being negated so that the tangent values are positive for the robot moving away from the wall
     *
     *  @param a the starting position
     *  @param b the desired position
     */
    private double getAngleBetween(Position a, Position b) {
        return -Math.atan2((b.y - a.y), (b.x - a.x));
    }

    /** Calculates the euclidean distance between two points.
     *
     *  @param a the starting position
     *  @param b the desired position
     */
    private double getEuclideanDistance(Position a, Position b) {
        return Math.sqrt(Math.pow(a.x - b.x, 2) + Math.pow(a.y - b.y, 2));
    }

    /** Transforms the robot's path based on the alliance color and side of the field it is starting on.
     */
    private void transformPath(RobotManager.StartingSide startingSide) {
        for (int i = 0; i < path.size(); i++) {
            Position pos = path.get(i);
            Position copy = new Position(pos.getX(),pos.getY(), pos.getName(),pos.getAction(),pos.getStrafePower(),pos.getRotatePower(),pos.getRotation());
//            if (allianceColor == RobotManager.AllianceColor.RED) {
//                copy.setY(-copy.getY());
//                copy.setRotation(-copy.getRotation());
//            }
            if (startingSide == RobotManager.StartingSide.RIGHT) {
                copy.setX(-copy.getX());
                copy.setRotation(-copy.getRotation());
            }
            path.set(i, copy);
        }
    }

    /**
     * Adds final parking location as the final position on the path.
     */
    private void setParkingLocation(RobotManager.StartingSide startingSide, RobotManager.ParkingPosition parking_position) {
        // TO DO: Add paths for three different paths
        //Parks in a signal parking spot to have a chance for 20 points
        if (parking_position == RobotManager.ParkingPosition.LEFT) {
            path.add(AutonomousPaths.leftBarcode); //No transformation occurs on this position so it will be the same
        } else if (parking_position == RobotManager.ParkingPosition.MIDDLE) {
            path.add(AutonomousPaths.centerBarcode);
        }
        else {
            path.add(AutonomousPaths.rightBarcode);
        }
    }

    /** Sets drive motor powers to make the robot move a certain way.
     *
     *  @param strafeDirection the direction in which the robot should strafe.
     *  @param power the speed at which the robot should strafe. Must be in the interval [-1, 1]. Set this to zero if
     *               you only want the robot to rotate.
     *  @param turn the speed at which the robot should rotate (clockwise). Must be in the interval [-1, 1]. Set this to
     *              zero if you only want the robot to strafe.
     */
    public void setDriveMotorPowers(double strafeDirection, double power, double turn, Robot robot, boolean debug) {
        for (RobotConfig.DriveMotors motor : RobotConfig.DriveMotors.values()) {
            Objects.requireNonNull(robot.driveMotors.get(motor)).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        robot.telemetry.addData("turn %.2f", turn);
        robot.telemetry.addData("current strafe direction", strafeDirection);
        if (Math.abs(power - 0) < FLOAT_EPSILON && Math.abs(turn - 0) < FLOAT_EPSILON) {
            stopMovement(robot);
            robot.telemetry.addData("stopping","YES");
        }else{
            robot.telemetry.addData("stopping","NO");
        }
        double sinMoveDirection = Math.sin(strafeDirection);
        double cosMoveDirection = Math.cos(strafeDirection);

        double powerSet1 = sinMoveDirection + cosMoveDirection;
        double powerSet2 = sinMoveDirection - cosMoveDirection;
        double [] rawPowers = scaleRange(powerSet1, powerSet2);

        robot.telemetry.addData("Front Motors", "left (%.2f), right (%.2f)",
                (rawPowers[0] * power + turn) * wheel_speeds[2], (rawPowers[1] * power - turn) * wheel_speeds[3]);
        robot.telemetry.addData("Rear Motors", "left (%.2f), right (%.2f)",
                (rawPowers[1] * power + turn) * wheel_speeds[0], (rawPowers[0] * power - turn) * wheel_speeds[1]);

        if (debug) {
            double start = robot.elapsedTime.milliseconds();
            while (robot.elapsedTime.milliseconds() - start > 100) {}
            return;
        }

        robot.driveMotors.get(RobotConfig.DriveMotors.REAR_LEFT).setPower((rawPowers[1] * power - turn) * wheel_speeds[0]);
        robot.driveMotors.get(RobotConfig.DriveMotors.REAR_RIGHT).setPower((rawPowers[0] * power + turn) * wheel_speeds[1]);
        robot.driveMotors.get(RobotConfig.DriveMotors.FRONT_LEFT).setPower((rawPowers[0] * power - turn) * wheel_speeds[2]);
        robot.driveMotors.get(RobotConfig.DriveMotors.FRONT_RIGHT).setPower((rawPowers[1] * power + turn) * wheel_speeds[3]);
    }

    /** Sets all drivetrain motor powers to zero.
     */
    public void stopMovement(Robot robot) {
        for (RobotConfig.DriveMotors motor : RobotConfig.DriveMotors.values()) {
            Objects.requireNonNull(robot.driveMotors.get(motor)).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Objects.requireNonNull(robot.driveMotors.get(motor)).setPower(0.0);
        }
    }

    /**preserves the ratio between a and b while restricting them to the range [-1, 1]
     *
     * @param a value to be scaled
     * @param b value to be scaled
     * @return an array containing the scaled versions of a and b
     */
    public double[] scaleRange(double a, double b) {
        double max = Math.max(Math.abs(a), Math.abs(b));
        return new double[] {a / max, b / max};
    }

/** Hardcoded paths through the playing field during the Autonomous period.
 */
public static class AutonomousPaths {
    public static final double TILE_SIZE = 23.5625;

    public static final double FIELD_WIDTH = 6;//placeholder - field is 6 x 6 square

    //Units are in field tiles.

    //Positions

    //Junctions
    public static Position nearHighJunction = new Position(1 * TILE_SIZE, -1.5*TILE_SIZE, "POI nearHighJunction", Action.DELIVER_CONE_HIGH, 1, 1, Math.PI / 2);
//    public static Position nearHighJunction = new Position(1 * TILE_SIZE, -1.5*TILE_SIZE, "POI nearHighJunction", Action.NONE, 1, 1, Math.PI / 2);
    public static Position farHighJunction = new Position(0.5 * TILE_SIZE,
            -2 * TILE_SIZE, "POI farHighJunction", Action.DELIVER_CONE_HIGH_90, 1, 1, Math.PI / 2);
//    public static Position farHighJunction = new Position(0.5 * TILE_SIZE,
//            -2 * TILE_SIZE, "POI farHighJunction", Action.NONE, 1, 1, Math.PI / 2);
    public static Position mediumJunction = new Position(0, -1.5*TILE_SIZE, "POI mediumJunction", Action.DELIVER_CONE_MEDIUM, 1, 1, 0);
    public static Position nearLowJunction = new Position(0.5*TILE_SIZE, 0, "POI nearLowJunction", Action.DELIVER_CONE_LOW, 1, 1, 0);
    public static Position farLowJunction = new Position(-0.5*TILE_SIZE, -TILE_SIZE, "POI farLowJunction", Action.DELIVER_CONE_LOW, 1, 1, 0);

    //Cone Stack
    public static Position coneStack1 = new Position(-0.75 * TILE_SIZE, -2 * TILE_SIZE, "POI coneStackFirstCone", Action.PICK_UP_FIRST_STACK_CONE, 1, 1, Math.PI / 2);
//    public static Position coneStack1 = new Position(-0.75 * TILE_SIZE, -2 * TILE_SIZE, "POI coneStackFirstCone", Action.NONE, 1, 1, Math.PI / 2);
    public static Position coneStack2 = new Position(-0.75 * TILE_SIZE, -2 * TILE_SIZE, "POI coneStackSecondCone", Action.PICK_UP_SECOND_STACK_CONE, 1, 1, Math.PI / 2);

    //Intermediate Locations. Since these values could be transformed, inner refers to the middle of the entire field, center
    // to the center of the left or right, and outer refers to the very edges of the field next to either side wall
    // Back refers to the tiles closest to the wall, front refers to the tiles furthest away from the wall/drivers
    public static Position intermediateInnerBack = new Position(TILE_SIZE, 0, 0, "intermediateInnerBack");
    public static Position intermediateCenterBack = new Position(0, 0, 0, "intermediateCenterBack");
    public static Position intermediateOuterBack = new Position(-TILE_SIZE, 0, 0, "intermediateOuterBack");
    public static Position intermediateInnerMiddle = new Position(TILE_SIZE, -TILE_SIZE, 0, "intermediateInnerMiddle");
    public static Position intermediateCenterMiddle = new Position(0, -TILE_SIZE, 0, "intermediateCenterMiddle");
    public static Position intermediateOuterMiddle = new Position(-TILE_SIZE, -TILE_SIZE, 0, "intermediateOuterMiddle");
    public static Position intermediateInnerFront = new Position(TILE_SIZE, -2*TILE_SIZE, Math.PI / 2, "intermediateInnerFront");
    public static Position intermediateCenterFront = new Position(0, -2*TILE_SIZE, 0, "intermediateCenterFront");
    public static Position intermediateOuterFront = new Position(0,-2 * TILE_SIZE, 0, "intermediateOuterFront");

    //Parking Locations
    //Make sure NOT to transform these in transformPath()
    public static Position leftBarcode = new Position(-TILE_SIZE, -1.5 * TILE_SIZE, 0, "leftBarcodeParkingPosition");
    public static Position centerBarcode = new Position(0, -1.5 * TILE_SIZE, 0, "centerBarcodeParkingPosition");
    public static Position rightBarcode = new Position(TILE_SIZE, -1.5 * TILE_SIZE, 0, "rightBarcodeParkingPosition");

    //ZE ULTIMATE PATH - Prepare for trouble! And make it double! To protect the world from devastation!
    //To unite all peoples within our nation! To denounce the evils of truth and love!
    public static final ArrayList<Position> CYCLE_HIGH = new ArrayList<>(Arrays.asList(
            intermediateInnerBack, nearHighJunction, intermediateInnerFront, coneStack1, farHighJunction));
//    public static final ArrayList<Position> CYCLE_HIGH = new ArrayList<>(Arrays.asList(
//            new Position(TILE_SIZE, 0, 0, "sussy test")));

}


}