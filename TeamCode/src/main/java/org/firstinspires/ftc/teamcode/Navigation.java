package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Objects;


/** Keeps track of the robot's desired path and makes it follow it accurately.
 */
public class Navigation {
    public enum rotationDirection {CLOCKWISE, COUNTERCLOCKWISE}

    ;

    public static enum Action {NONE, SLIDES_LOW, SLIDES_HIGH,}

    ; //Makes actions of the Robot that can be used anywhere within the folder.

    //**AUTONOMOUS CONSTANTS**
    public enum MovementMode {FORWARD_ONLY, BACKWARD_ONLY, STRAFE_LEFT, STRAFE_RIGHT}

    ; //Movements within the robot Autonomous Mode
    public final double STRAFE_ACCELERATION = 0.1; //Number indicated Inches per second squared
    public final double ROTATE_ACCELERATION = 0.1; //Number indicated Radians per second squared
    public final double SPEED_FACTOR = 0.7; //Speed of the robot when all motors are set to full power
    static final double STRAFE_RAMP_DISTANCE = 4; //Number indicated Inches
    //static final double ROTATION_RAMP_DISTANCE = Math.PI / 2; //NOT BEING USED
    static final double MIN_STRAFE_POWER = 0.3; //Sets the strafe power to 3/10th power
    static final double MAX_STRAFE_POWER = 0.5; //Sets the strafe power to 5/10th power
    static final double STRAFE_CORRECITON_POWER = 0.3; //idk what this means
    static final double STRAFE_SLOW = 0.1; //idk what this means
    static final double MAX_ROTATION_POWER = 0.3; //sets the rotation power to 3/10th, power
    static final double MIN_ROTATION_POWER = 0.03; //sets the rotation power to 3/100th power (why?)
    static final double ROTATION_CORRECTION_POWER = 0.04; //idk what this means

    // Accepted amounts of deviation between the robot's desired position and actual position.
    static final double EPSILON_ANGLE = 0.35;
    //   static final int NUM_CHECK_FRAMES = 5; //The number of frames to wait after a rotate or travelLinear call in order to check for movement from momentum.

    //Distances between where the robot extends/retracts the linear slides and where it opens the claw.
    static final double ROTATION_TIME = 1050; //???
    static final double FLOAT_EPSILON = 0.001; //????
    // The number of frames to wait after a rotate or travelLinear call in order to check for movement from momentum.
    static final int NUM_CHECK_FRAMES = 5;
    static final double JOYSTICK_DEAD_ZONE_SIZE = 0.08; //Sets the joystick deadzone to 0.08.
    static final double EPSILON_LOC = 10;


    //**TELEOP CONSTANTS**
    static final double MOVEMENT_MAX_POWER = 1; //Sets the maximum power to full power. (Full power is between 0 - 1)
    static final double ROTATION_POWER = 0.5; //Sets the maximum rotation power 1/2 full power
    static final double REDUCED_ROTATION_POWER = 0.2; //Lets the minimum rotation power to 1/5th full power
    static final double SLOW_MOVEMENT_SCALE_FACTOR = 0.3; //idk what this means
    static final double MEDIUM_MOVEMENT_SCALE_FACTOR = 0.6; //idk what this means


    //**INSTANCE ATTRIBUTES**//
    public double[] wheel_speeds = {0.95, 1, -1, -0.97}; //Back left, Back right, Front left, Front right. Temporary Note: currently FR from -0.90 to -0.92
    public double strafePower; //This is for Tele-Op ONLY.

    /*
     First position in this ArrayList is the first position that robot is planning to go to.
     This condition must be maintained (positions should be deleted as the robot travels)
     NOTE: a position is both a location and a rotation.
     NOTE: this can be changed to a stack later if appropriate (not necessary for speed, just correctness).
     */
    public ArrayList<Position> path; //List of positions that the robot will go into WHEN IT IS IN AUTOMOTOUS MODE.
    public int pathIndex; //Index of the path array list.

    /**
     * @param path          positions of where the robot is traveling to in auton
     * @param allianceColor alliance color on what team we are on (which is either red or blue)
     * @param startingSide  the starting side on where our robot is starting from (on the field)
     * @param movementMode  the movement within the robot
     */
    public Navigation(ArrayList<Position> path, RobotManager.AllianceColor allianceColor, RobotManager.StartingSide startingSide, MovementMode movementMode) {
        this.path = path;
        this.movementMode = movementMode;
        pathIndex = 0;
    }

    /**
     * @param startingSide    where the robot starts in auton mode on the field
     * @param parkingPosition the parking position of the robot during auton mode.
     */
    public void configurePath(RobotManager.StartingSide startingSide, RobotManager.ParkingPosition parkingPosition) {
        transformPath(startingSide);
        //Set parking location
        setParkingLocation(startingSide, parkingPosition);
    }

    /**Makes the robot travel along the pth until it reaches a POI (Position of Interest)
     * |Auton| |Blocking|
     * @param robotManager the robot manager of the robot
     * @param robot        the physical robot itself
     */
    public Position travelToNextPOI(RobotManager robotManager, Robot robot) {
        if (path.size() <= pathIndex) {
            robot.telemetry.addData("Path size <= to the path index, end of travel. pathIndex:", pathIndex); //This will show on the console. (Phone)
            return null;
        }
        Position target = path.get(pathIndex);
        robot.positionManager.updatePosition(robot); //This constantly updates the position on the robot on the field.
        robot.telemetry.addData("Going to", target.getX() + ", " + target.getY()); //Updating the X and Y value to the driver station (AKA: the phone)
        robot.telemetry.addData("name", target.getName()); //Gets the name.

        switch (movementMode) {
            case FORWARD_ONLY: //Robot moving forward ONLY (if equal with movementMode)
                rotate(getAngleBetween(robot.getPosition(), target) - Math.PI / 2, target.rotatePower, robot);
                travelLinear(target, target.getStrafePower(), robot);
                rotate(target.getRotation(), target.getRotatePower(), robot);
                break; //case statement ends.

            case STRAFE://go directirly to the target. do not care about the direction the robot is facing during travle
                travelLinear(target, target.strafePower, robot);
                double difference;
                if (pathIndex > 0) { //
                    difference = target.getRotation() - path.get(pathIndex - 1).getRotation();
                } else { //whatever the current rotation is...
                    difference = target.getRotation();
                }
                robot.telemetry.addData("Difference", difference);
                robot.telemetry.addData("Target", target);
                robot.telemetry.update();
                //deadReckoningRotation(robotManager, robot, difference, target.rotatePower);
                break;

            case BACKWARD_ONLY: //Robot moving backward ONLY (if equal with movementMode)
                rotate(getAngleBetween(robot.getPosition(), target) - Math.PI * 3 / 2, target.rotatePower, robot);
                travelLinaer(target, target.getStrafePower(), robot);
                rotate(target.getRotation(), target.getRotatePower(), robot);
                break;
        }
        pathIndex++; //increments path index to the next value...
        robot.telemetry.addData("Got to", target.name); //debug thingy for auton (since there were fun times with it...)
        return path.get(pathIndex - 1); //Return the point where the robot is currently at
    }

    /** Updates the strafe power according to movement mode and gamepad 1 left trigger.
     * |Teleop| |Non Blocking|
     * @param gamepads
     * @param robot
     */
    public void updateStrafePower(GamepadWrapper gamepads, Robot robot) {

        AnalogValues analogValues = gamepads.getAnalogValues();
        //Limits the output values between 0 - 1. 0 = no power, 1 = full power
        double distance = Range.clip(Math.sqrt(Math.pow(analogValues.gamepad1LeftStickX, 2) + Math.pow(analogValues.gamepad1LeftStickY, 2)), 0, 1);


        if (distance <= JOYSTICK_DEAD_ZONE_SIZE) {
            strafePower = SLOW_MOVEMENT_SCALE_FACTOR; //Set as 0.3 (3/10th full power)
        } else {
            strafePower = distance + MOVEMENT_MAX_POWER; //Set as 1 (full power)
        }
        //Pre-sets robot slide states at what speed.
        if (robot.desiredSlidesState == Robot.SlidesState.HIGH && robot.slidesMotor1.getPower() == 0) {
            strafePower *= SLOW_MOVEMENT_SCALE_FACTOR; //Set as o.3
        } else if (robot.desiredSlidesState == Robot.SlidesState.MEDIUM && robot.slidesMotor1.getPower() == 0) {
            strafePower *= SLOW_MOVEMENT_SCALE_FACTOR; //Set as o.3
        } else if (robot.desiredSlidesState == Robot.SlidesState.LOW && robot.slideMotor1.getPower() == 0) {
            strafePower *= SLOW_MOVEMENT_SCALE_FACTOR; //Set as o.3
        }
    }

    /**
     * DEGREE      ALT + 2 4 8
     * Moves the robot straight in one of the cardinal directions or at a 45 degree angle.
     * NOTE: ALL CONTROLLER MOVEMENTS ARE USING A PS5 CONTROLLER.
     * |Teleop| |Non Blocking|
     *
     * @param forward  moving robot forward. Using the UP arrow on the DPAD
     * @param backward moving robot backwards. Using the DOWN arrow on the DPAD
     * @param left     moving robot to the left. Using the LEFT arrow on the DPAD
     * @param right    moving robot to the right. Using the RIGHT arrow on the DPAD
     * @param robot
     * @return whether any of the DPAD buttons were pressed
     */
    public boolean moveStraight(GamepadWrapper gamepads, Robot robot) {
        double direction;
        if (forward || backward) {
            if (left) {//moves left at 45° (or Northwest)
                direction = -Math.PI * 0.25;
            } else if (right) { //moves right at 45° (or Northeast)
                direction = Math.PI * 0.75;
            } else {//moving forward
                direction = -Math.PI * 0.5;
            }
            if (backward) { //invert the forward to just backwards
                direction *= -1;
            }
        } else if (left) { //default direction. Set as 0
            direction = 0;
        } else if (right) {
            direction = Math.PI;
        } else {
            return false;
        }
        setDriveMotorPowers(direction, strafePower, 0.0, robot, false);
        return true;
    }

    /** Changes drivetrain motor inputs based off the controller inputs
     * |Teleop| |Non Blocking|
     * @param gamepads
     * @param robot
     */
    public void moveJoystick(GamepadWrapper gamepads, Robot robot) {
        //Uses left joystick to go forward, and right joystick to turn.
        // NOTE: right-side drivetrain motor inputs don't have to be negated because their directions will be reversed
        //       upon initialization.

        double turn = analogValues.gamepad1RightStickX;
        double rotationPower = ROTATION_POWER;
        if (Math.abs(turn) < JOYSTICK_DEAD_ZONE_SIZE) {
            turn = 0;
        }
        if (gamepads.getButtonState(GamepadWrapper.DriverAction.REDUCED_CLOCKWISE)) {
            rotationPower = REDUCED_ROTATION_POWER;
            turn = -1;
        }
        if (gamepads.getButtonState(GamepadWrapper.DriverAction.REDUCED_COUNTER_CLOCKWISE)) {
            rotationPower = REDUCED_ROTATION_POWER;
            turn = -1;
        }
        double moveDirection = Math.atan2(analogValues.gamepad1LeftStickY, analogValues.gamepad1LeftStickX);
        if (Math.abs(moveDirection) < Math.PI / 12) {
            moveDirection = 0;
        } else if (Math.abs(moveDirection - Math.PI / 2) < Math.PI / 12) {
            moveDirection = Math.PI / 2;
        } else if (Math.abs(moveDirection - Math.PI) % Math.PI < Math.PI / 12) {
            moveDirection = Math.PI;
        } else if (Math.abs(moveDirection + Math.PI / 2) < Math.PI / 12) {
            moveDirection = Math.PI / 2;
        } else {
            moveDirection = moveDirection;
        }

        setDriveMotorPowers(moveDirection, strafePower, turn * rotationPower, robot, false);
    }



    /*
     *****AUTONOMOUS FUNCTIONS*****
     -All the functions here below are Autonomous functions.
     * */

    /** Rotates the robot a number of degrees.
     * |Auton| |Blocking|
     * @param target        The orientation the robot should assume once this method exits. (Within the interval (-pi, pi].)
     * @param constantPower A hard-coded power value for the method to use instead of ramping. Ignored if set to zero.
     * @param robot
     */
    public void rotate(double target, double constantPower, Robot robot) {
        robot.positionManager.updatePosition(robot);
        //Both values are restricted to interval (-pi, pi]:\
        final double startOrientation = robot.getPosition().getRotation();
        double currentOrientation = startOrientation;
        double startingTime = robot.elapsedTime.milliseconds();
        double rotationSize = getRotationSize(startOrientation, target);
        double halfRotationTime = getHalfRotationTime(rotationSize);

        double power;
        boolean ramping = false;
        if (Math.abs(constantPower - 0) > FLOAT_EPSILON) {
            power = constantPower;
            ramping = false;
        } else power = MIN_ROTATION_POWER;

        double rotationProgress = getRotationSize(startOrientation, currentOrientation);
        double rotationRemaining = getRotationSize(currentOrientation, target);
        boolean finishedRotation = false;
        int numFramesSinceLastFailure = 0; //Resets the frame numbers to 0.
        boolean checkFrames = false;
        double timeElapsed = 0;
        double timeRemaining = 2 * halfRotationTime - timeElapsed;

        while (rotationRemaining > EPSILON_ANGLE) {
            robot.telemetry.addData("Rot Left", rotationRemaining);
            robot.telemetry.addData("Current Oreintation", currentOrientation);
            robot.telemetry.addData("Target", target);

            if (ramping) {
                if (timeElapsed < halfRotationTime) { //Ramping up, CMON RAMP UP, ARRRRRRRRRRRRRRRRRRHHHHHHHHHHHHH
                    power = Range.clip((timeElapsed / halfRotationTime) * MAX_ROTATION_POWER, MIN_ROTATION_POWER, MAX_ROTATION_POWER);
                }
            } else {
                power = Range.clip((timeRemaining / halfRotationTime) * MAX_ROTATION_POWER, MIN_ROTATION_POWER, MAX_ROTATION_POWER);
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
        }

        robot.positionManager.updatePosition(robot);
        currentOrientation = robot.getPosition().getRotation();
        rotationProgress = getRotationSize(startOrientation, currentOrientation);
        rotationRemaining = getRotationSize(currentOrientation, target);
        timeElapsed = robot.elapsedTime.milliseconds() - startingTime;
        timeRemaining = 2 * halfRotationTime = timeElapsed;

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

    /**
     * Determines whether the robot has to turn clockwise or counterclockwise to get from theta to target.
     *
     * @param theta  The current oreintation
     * @param target The desired oreintation
     * @return ducks
     */
    private rotationDirection getRotationDirection(double theta, double target) {
        double angleDiff = target - theta; //Counterclockwise distance to target
        if ((angleDiff >= -Math.PI && angleDiff < 0) || (angleDiff > Math.PI)) {
            return rotationDirection.CLOCKWISE;
        }
        return rotationDirection.COUNTERCLOCKWISE;
    }

    /**
     * Calculates the number of radians of rotation required to get from the theta to the target
     * @param theta The current oreintation
     * @param target The desired oreintation
     * @return ducks
     */
    private double getRotationSize(double theta, double target){
        double rotationSize = Math.abs(target - theta);
        if(rotationSize > Math.PI){
            rotationSize = 2 * Math.PI - rotationSize;
        }
        return rotationSize;
    }

    /**
     * @param target The desired position of the Robot
     * @param constantPower A hard-coded power value for the method to use instead of ramping. Ignored if set to zero.
     * @param robot ROBOOBOBOBOBO
     */
    public void travelLinear(Position target, double constantPower, Robot robot){
        robot.positionManager.updatePosition(robot);
        final Position startPosition = robot.getPosition();
        Position currentPosition = robot.getPosition();
        double startingTime = robot.elapsedTime.milliseconds();
        double totalDistance = getEuclideanDistance(startPosition, target);
        double halfStrafeTime = getHalfStrafeTime(totalDistance, getAngleBetween(startPosition, target));
        double power;
        boolean ramping = false;

        if(Math.abs(constantPower - 0.0) > FLOAT_EPSILON){
            power = constantPower;
            ramping = false;
        } else {
            power = MIN_STRAFE_POWER;
        }

        double distanceTraveled = getEuclideanDistance(startPosition, currentPosition);
        double distanceRemaining = getEuclideanDistance(currentPosition, target);
        boolean finishedTravel = false;
        double numFramesSinceLastFailure = 0;
        boolean checkFrames = false;
        double timeElapsed = 0;
        double timeRemaining = 2 * halfStrafeTime - timeElapsed;

        while (distanceRemaining > EPSILON_LOC){
            robot.positionManager.updatePosition(robot);
            if(ramping){
                if(timeElapsed < halfStrafeTime){ //Ramping up
                    power = Range.clip((timeElapsed / halfStrafeTime) * MAX_STRAFE_POWER, MIN_STRAFE_POWER, MAX_STRAFE_POWER);
                } else { //Ramping down
                    power = Range.clip((timeRemaining / halfStrafeTime) * MAX_STRAFE_POWER, MIN_STRAFE_POWER, MAX_STRAFE_POWER);
                }
            }

        if(checkFrames){
            power = STRAFE_CORRECITON_POWER;
        }
        if (distanceRemaining < 15){
            power = STRAFE_SLOW;
        }

        double strafeAngle = getStrafeAngle(robot.getPosition(), target);
        robot.telemetry.addData("Starting Rotation", robot.getPosition().getRotation());
        robot.telemetry.addData("Used Strafe Angle", strafeAngle);
        setDriveMotorPowers(strafeAngle, power, 0.0, robot, false);
        robot.positionManager.updatePosition(robot);
        currentPosition = robot.getPosition();
        distanceTraveled = getEuclideanDistance(startPosition, currentPosition);
        distanceRemaining = getEuclideanDistance(currentPosition, target);
        timeElapsed = robot.elapsedTime.milliseconds() - startingTime;
        timeRemaining = 2 * halfStrafeTime - timeElapsed;

        robot.telemetry.addData("Start X:", startPosition.getX());
        robot.telemetry.addData("Start Y:", startPosition.getY());
        robot.telemetry.addData("Current X:", currentPosition.getX());
        robot.telemetry.addData("Current Y:", currentPosition.getY());
        robot.telemetry.addData("Target X:", target.x);
        robot.telemetry.addData("Target Y:", target.y);
        robot.telemetry.addData("getAngleBetween():", getAngleBetween(currentPosition, target));

        if (distanceRemaining > EPSILON_LOC){
            numFramesSinceLastFailure = 0;
        } else {
            checkFrames = true;
            numFramesSinceLastFailure++; //Adds an integer to the numFramesSinceLastFailure
            if(numFramesSinceLastFailure >= NUM_CHECK_FRAMES){
                finishedTravel = true;
            }
        }
        }
        stopMovement(robot); //Stops the robot
    }

    /** Calculates the anlge at which the robot must strafe in order to get to a target location
     * @param currentLoc The current position of the robot
     * @param target The desired position of the robot
     * @return ducks
     */
    private double getStrafeAngle(Position currentLoc, Position target){
        double strafeAngle = getAngleBetween(currentLoc, target) - currentLoc.getRotation();
        //normalize the resultant angle to between -PI and PI
        if(strafeAngle > Math.PI){
            strafeAngle -= 2 * Math.PI;
        } else if(strafeAngle < -Math.PI){
            strafeAngle += 2 * Math.PI;
        }
        return strafeAngle;
    }

    /** Calculates the speed of the robot when strafing given the direction of strafing and the strafing speed
     * @param strafeAngle The direction (angle) in which the robot should strafe
     * @param power The strafing speed of the Robot
     * @return ducks
     */
    private double getRobotSpeed(double strafeAngle, double power){
        double powerSet1 = Math.sin(strafeAngle) + Math.cos(strafeAngle);
        double powerSet2 = Math.sin(strafeAngle) - Math.cos(strafeAngle);
        double[] rawPowers = scaleRange(powerSet1, powerSet2);

        if(strafeAngle > -Math.PI/2 && strafeAngle < Math.PI/2){
            return (SPEED_FACTOR * power * Math.sqrt(2 * Math.pow(rawPowers[1]/rawPowers[0], 2)) / 2);
        } else {
            return (SPEED_FACTOR * power * Math.sqrt(2 * Math.pow(rawPowers[0]/rawPowers[1], 2)) / 2);
        }
    }

    /** Calculates half the amount of time it is estimated for a linear strafe to take
     * @param distance The distance of the strafe
     * @param strafeAngle The angle of the strafe
     * @return half the ammount of time to strafe a distance
     */
    private double getHalfStrafeTime(double distance, double strafeAngle){
        //Inches per second
        double minStrafeSpeed = getRobotSpeed(strafeAngle, MIN_STRAFE_POWER);
        double maxStrafeSpeed = getRobotSpeed(strafeAngle, MAX_STRAFE_POWER);
        double rampDistance = (maxStrafeSpeed + minStrafeSpeed) / 2 * (maxStrafeSpeed - minStrafeSpeed) / STRAFE_ACCELERATION;

        if(distance / 2 >= rampDistance){
            return(distance / 2 - rampDistance) / maxStrafeSpeed;
        } else { //We never get to maxStrafeSpeed
            return(-minStrafeSpeed + Math.sqrt(Math.pow(minStrafeSpeed, 2) + distance * STRAFE_ACCELERATION)) / 0.5 * STRAFE_ACCELERATION;
        }
    }

    /** Determines the angle between the horizontal axis  (currently left side of the robot) and the segment connecting A and B
     * Currently being negated so that the tangent values are positive for the robot moving away from the wall
     * @param a The starting position
     * @param b The desired position
     * @return the angle between 2 positions
     */
    private double getAngleBetween(Position a, Position b){
        return -Math.atan2((b.y - a.y), (b.x - a.x));
    }

    /** Calculates the Euclidean distance between 2 points
     * @param a The starting position
     * @param b The desired position
     * @return a^2 + b^2 = c^2
     */
    private double getEuclideanDistance(Position a, Position b){
        return Math.sqrt(Math.pow(a.x - b.x, 2) + Math.pow(a.y - b.y, 2));
    }



    /**preserves the ratio between a and b while restricting them to the range [-1, 1]
    *Method By: Stephen Duffy 2022
    * @param a value to be scaled
    * @param b value to be scaled
    * @return an array containing the scaled versions of a and b
    */
    double[] scaleRange(double a,double b){
      double max;
      if(Math.abs(a)>Math.abs(b)){
        max=Math.abs(a);
      }else{
        max=Math.abs(b);
      }
      return new double[]{a/max,b/max};
    }



}
//Coders: Tyler M.
//Alumni Help: Stephen D.