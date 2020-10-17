package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.teamcode.DriveModule.RotateModuleMode.DO_NOT_ROTATE_MODULES;
import static org.firstinspires.ftc.teamcode.DriveModule.RotateModuleMode.ROTATE_MODULES;

enum ModuleSide {LEFT, RIGHT}

public class DriveController {
    Robot robot;

    DriveModule moduleLeft;
    DriveModule moduleRight;

    Position robotPosition;

    DataLogger dataLogger;
    boolean debuggingMode;

    //used for straight line distance tracking
    double robotDistanceTraveled = 0;
    double previousRobotDistanceTraveled = 0;
    double moduleLeftLastDistance;
    double moduleRightLastDistance;

    final double WHEEL_TO_WHEEL_CM = 32.5; //in cm (was 18*2.54)

    //tolerance for module rotation (in degrees)
    public final double ALLOWED_MODULE_ROT_ERROR = 5;

    //tolerance for robot rotation (in degrees)
    public final double ALLOWED_ROBOT_ROT_ERROR = 5; //was 3

    //distance from target when power scaling will begin
    public final double START_DRIVE_SLOWDOWN_AT_CM = 15;

    //maximum number of times the robot will try to correct its heading when rotating
    public final int MAX_ITERATIONS_ROBOT_ROTATE = 2;

    //minimum drive power (ever)
    //TODO: actually set this to minimum possible power
    double MIN_DRIVE_POWER = 0.3;

    //will multiply the input from the rotation joystick (max value of 1) by this factor
    public final double ROBOT_ROTATION_SCALE_FACTOR = 0.7;
    public final double ROBOT_ROTATION_WHILE_TRANS_SCALE_FACTOR = 0.2;
    public final double ROBOT_ROTATION_SCALE_FACTOR_ABS = 1;
    public final double ROBOT_ROTATION_WHILE_TRANS_SCALE_FACTOR_ABS = 1;


    //default timeouts
    public double DEFAULT_TIMEOUT_ROT_MODULES = 750; //was 500
    public double ROTATE_ROBOT_TIMEOUT = 3000;
    public double DRIVE_TIMEOUT = 4000;

    //drive to position constants
    //todo: tune these constants
    double MAX_AUTO_DRIVE_FACTOR = .7; //was 1
    double MIN_AUTO_DRIVE_FACTOR = 0.1;
    double MAX_AUTO_ROTATE_FACTOR = 0.3; //was 0.5
    double MIN_AUTO_ROTATE_FACTOR = 0.1;

////    //Vuforia field tracking tools:
//    //This is the allowed distance to a target
//    public final double ALLOWED_DISTANCE_TO_TARGET = 5; //todo change this value
//    //This is the object to get the position on field
//    public FieldTracker vuforiaTracker;

    public DriveController(Robot robot, Position startingPosition, boolean debuggingMode) {
        this.robot = robot;
        this.debuggingMode = debuggingMode;
        moduleLeft = new DriveModule(robot, ModuleSide.LEFT, debuggingMode);
        moduleRight = new DriveModule(robot, ModuleSide.RIGHT, debuggingMode);

        //todo: change to parameter
        robotPosition = startingPosition;

        //vuforiaTracker = new FieldTracker(robot.hardwareMap, robot.telemetry, true, false);

        if (debuggingMode) {
            dataLogger = new DataLogger("Drive Controller");
            dataLogger.addField("X Position");
            dataLogger.addField("Y Position");
            dataLogger.addField("X Power");
            dataLogger.addField("Y Power");
            dataLogger.addField("Rotation Power");
            dataLogger.addField("Translation Direction X");
            dataLogger.addField("Translation Direction Y");
            dataLogger.addField("Rotation Direction");
            dataLogger.addField("Translation Vector X");
            dataLogger.addField("Translation Vector Y");
            dataLogger.newLine();
        }
    }

    //defaults to debugging mode off, starting position of 0, 0
    public DriveController(Robot robot) {
        this(robot, new Position(0, 0, new Angle(0, Angle.AngleType.ZERO_TO_360_HEADING)), false);
    }

    //defaults to debugging mode off
    public DriveController(Robot robot, Position startingPosition) {
        this(robot, startingPosition, false);
    }

    //converts joystick vectors to parameters for update() method
    //called every loop cycle in TeleOp
    public void updateUsingJoysticks(Vector2d joystick1, Vector2d joystick2, boolean absHeadingMode) {
        if (absHeadingMode) {
            if (joystick1.getMagnitude() == 0)
                updateAbsRotation(joystick1, joystick2, ROBOT_ROTATION_SCALE_FACTOR_ABS);
            else updateAbsRotation(joystick1, joystick2, ROBOT_ROTATION_WHILE_TRANS_SCALE_FACTOR_ABS);
        } else {
            if (joystick1.getMagnitude() == 0) update(joystick1, -joystick2.getX() * ROBOT_ROTATION_SCALE_FACTOR);
            else update(joystick1, -joystick2.getX() * ROBOT_ROTATION_WHILE_TRANS_SCALE_FACTOR);
        }
    }

    //should be called every loop cycle when driving (auto or TeleOp)
    //note: positive rotationMagnitude is CCW rotation
    //this method is for "power-based rotation mode"
    public void update(Vector2d translationVector, double rotationMagnitude) {
        moduleLeft.updateTarget(translationVector, rotationMagnitude);
        moduleRight.updateTarget(translationVector, rotationMagnitude);
    }
    public void updateAbsRotation(Vector2d translationVector, Vector2d joystick2, double scaleFactor) {
        Angle targetAngle = joystick2.getAngle(); //was + .convertAngle(Angle.AngleType.NEG_180_TO_180_HEADING)
        if (joystick2.getMagnitude() > 0.1 && targetAngle.getDifference(robot.getRobotHeading()) > 3) {
            moduleLeft.updateTargetAbsRotation(translationVector, targetAngle, scaleFactor);
            moduleRight.updateTargetAbsRotation(translationVector, targetAngle, scaleFactor);
        } else {
            moduleLeft.updateTarget(translationVector, 0);
            moduleRight.updateTarget(translationVector, 0);
        }
    }

    //AUTONOMOUS METHODS
    //do NOT call in a loop

    //speed should be scalar from 0 to 1
    public void drive(Vector2d direction, double cmDistance, double speed, boolean fixModules, boolean alignModules, LinearOpMode linearOpMode) {
        cmDistance = cmDistance/2.0; //BAD :(
        double startTime = System.currentTimeMillis();
        double initalSpeed = speed;

        //turns modules to correct positions for straight driving
        if (alignModules) rotateModules(direction, true, DEFAULT_TIMEOUT_ROT_MODULES, linearOpMode);

        //sets a flag in modules so that they will not try to correct rotation while driving
        if (fixModules) setRotateModuleMode(DO_NOT_ROTATE_MODULES);
        else setRotateModuleMode(ROTATE_MODULES); //reset mode

        resetDistanceTraveled();
        //updateTracking(); //ADDED

        while (getDistanceTraveled() < cmDistance && /*System.currentTimeMillis() - startTime < DRIVE_TIMEOUT && */linearOpMode.opModeIsActive()) {
            //slows down drive power in certain range
            if (cmDistance - getDistanceTraveled() < START_DRIVE_SLOWDOWN_AT_CM) {
                speed = RobotUtil.scaleVal(cmDistance - getDistanceTraveled(), 0, START_DRIVE_SLOWDOWN_AT_CM, MIN_DRIVE_POWER, initalSpeed);
                linearOpMode.telemetry.addData("speed: ", speed);
            }
            updateTracking(); //WAS MOVED ABOVE
            update(direction.normalize(Math.abs(speed)), 0); //added ABS for DEBUGGING

            linearOpMode.telemetry.addData("Driving robot", "");
            linearOpMode.telemetry.update();

            updatePositionTracking(robot.telemetry); //update position tracking
        }
        update(Vector2d.ZERO, 0);
        setRotateModuleMode(ROTATE_MODULES); //reset mode
    }

    //aligns modules before driving but DOES NOT fix them (modules will adjust orientation while driving)
    public void drive(Vector2d direction, double cmDistance, double speed, LinearOpMode linearOpMode) {
        drive(direction, cmDistance, speed, false, true, linearOpMode);
    }


//    public void driveWhileAdjusting(double distance) {
//        while (getDistanceTraveled() < distance) {
//            double distanceDifference = moduleRight.getDistanceTraveled() - moduleLeft.getDistanceTraveled();
//            double powerDifference = RobotUtil.mapToLineThroughOrigin(distanceDifference, 0.4, 30);
//            //TODO change the values above to constants and tune
//            double averagePower = RobotUtil.scaleVal(distance - getDistanceTraveled(), minDistance, maxDistance, minPower, maxPower);
//            moduleLeft.driveWhileAdjusting(averagePower - powerDifference / 2);
//            moduleRight.driveWhileAdjusting(averagePower + powerDifference / 2);
//            //TODO account for the fact that power could be over 1 and scale
//        }
//    }

    //speed should be scalar from 0 to 1
    public void driveWithRange (Vector2d direction, double stopAtDistance, boolean forward, boolean frontSensor, double speed, double timeout, boolean fixModules, boolean alignModules, LinearOpMode linearOpMode) {
        if (frontSensor) stopAtDistance = stopAtDistance + 10; //account for inset into robot
        double initalSpeed = speed;

        //turns modules to correct positions for straight driving
        if (alignModules) rotateModules(direction, true, DEFAULT_TIMEOUT_ROT_MODULES, linearOpMode);

        //sets a flag in modules so that they will not try to correct rotation while driving
        if (fixModules) setRotateModuleMode(DO_NOT_ROTATE_MODULES);
        boolean continueLoop;

        double startTime = System.currentTimeMillis();
        resetDistanceTraveled();

        do {
            //loop stop condition
            if (forward) continueLoop = 0 > stopAtDistance;
            else continueLoop = 1000000 < stopAtDistance;

            //slows down drive power in certain range
            updateTracking();
            update(direction.normalize(Math.abs(speed)), 0); //added ABS for DEBUGGING

            linearOpMode.telemetry.addData("Driving robot", "");
            linearOpMode.telemetry.update();
            updatePositionTracking(robot.telemetry); //update position tracking
        } while (continueLoop && System.currentTimeMillis() - startTime < DRIVE_TIMEOUT && System.currentTimeMillis() - startTime < timeout && linearOpMode.opModeIsActive());

        update(Vector2d.ZERO, 0);
        setRotateModuleMode(ROTATE_MODULES); //reset mode
    }

    //defaults to fix modules and align modules both TRUE
    public void driveWithRange(Vector2d direction, double stopAtDistance, boolean forward, boolean frontSensor, double speed, double timeout, LinearOpMode linearOpMode) {
        driveWithRange(direction, stopAtDistance, forward, frontSensor, speed, timeout,true, true, linearOpMode);
    }

    //speed should be scalar from 0 to 1
    public void driveWithTimeout(Vector2d direction, double cmDistance, double speed, double timeout, boolean fixModules, boolean alignModules, LinearOpMode linearOpMode) {
        cmDistance = cmDistance/2.0; //BAD :(
        double startTime = System.currentTimeMillis();
        double initalSpeed = speed;

        //turns modules to correct positions for straight driving
        if (alignModules) rotateModules(direction, true, DEFAULT_TIMEOUT_ROT_MODULES, linearOpMode);

        //sets a flag in modules so that they will not try to correct rotation while driving
        if (fixModules) setRotateModuleMode(DO_NOT_ROTATE_MODULES);
        else setRotateModuleMode(ROTATE_MODULES); //reset mode

        resetDistanceTraveled();
        //updateTracking(); //ADDED

        while (getDistanceTraveled() < cmDistance && System.currentTimeMillis() - startTime < timeout && linearOpMode.opModeIsActive()) {
            //slows down drive power in certain range
            if (cmDistance - getDistanceTraveled() < START_DRIVE_SLOWDOWN_AT_CM) {
                speed = RobotUtil.scaleVal(cmDistance - getDistanceTraveled(), 0, START_DRIVE_SLOWDOWN_AT_CM, MIN_DRIVE_POWER, initalSpeed);
                linearOpMode.telemetry.addData("speed: ", speed);
            }
            updateTracking(); //WAS MOVED ABOVE
            update(direction.normalize(Math.abs(speed)), 0); //added ABS for DEBUGGING

            linearOpMode.telemetry.addData("Driving robot", "");
            linearOpMode.telemetry.update();
            updatePositionTracking(robot.telemetry); //update position tracking
        }
        update(Vector2d.ZERO, 0);
        setRotateModuleMode(ROTATE_MODULES); //reset mode
    }

    //aligns modules before driving but DOES NOT fix them (modules will adjust orientation while driving)
    public void driveWithTimeout(Vector2d direction, double cmDistance, double speed, double timeout, LinearOpMode linearOpMode) {
        driveWithTimeout(direction, cmDistance, speed, timeout, false, true, linearOpMode);
    }


    //position tracking drive method
    public void driveToPosition(Position targetPosition, boolean isBlue, LinearOpMode linearOpMode) {
        double totalTravelDistance = robotPosition.getVectorTo(targetPosition).getMagnitude();
        double totalHeadingDifference = robotPosition.getAbsHeadingDifference(targetPosition);

        do {
            //scale speeds based on remaining distance from target, bounded by 0 and original distance
            //at the very beginning, x & y trans and rot will be max speed (different for each)
            //at the end, all three speeds will be min speed (different for each)
            //in between, the speeds will scale linearly depending on distance from target position
            robot.updateBulkData();
            updatePositionTracking(linearOpMode.telemetry);

            Vector2d translationDirection = robotPosition.getDirectionTo(targetPosition);
            Angle.Direction rotationDirection = robotPosition.getRotationDirectionTo(targetPosition);

            double rotationPower = RobotUtil.scaleVal(robotPosition.getAbsHeadingDifference(targetPosition),
                    0, totalHeadingDifference, 0, MAX_AUTO_ROTATE_FACTOR);

            if (robotPosition.getRotationDirectionTo(targetPosition) == Angle.Direction.CLOCKWISE) {
                rotationPower *= -1; //todo: check sign
            }

            double distanceRemaining = robotPosition.getVectorTo(targetPosition).getMagnitude();
            double translationScaleFactor = RobotUtil.scaleVal(distanceRemaining, 0, totalTravelDistance, 0, 1); //min output was 0.1
            Vector2d translationVector = translationDirection.scale(translationScaleFactor);
            if (isBlue) translationVector = translationVector.reflect();

            update(translationVector, rotationPower);

            if (debuggingMode) {
                dataLogger.addField(robotPosition.x);
                dataLogger.addField(robotPosition.y);
                dataLogger.addField(rotationPower);
                dataLogger.addField(translationDirection.getX());
                dataLogger.addField(translationDirection.getY());
                dataLogger.addField(rotationDirection.toString());
                dataLogger.addField(translationVector.getX());
                dataLogger.addField(translationVector.getY());
                dataLogger.newLine();

                linearOpMode.telemetry.addData("Distance remaining", distanceRemaining);
                linearOpMode.telemetry.addData("Translation direction", translationDirection);
                linearOpMode.telemetry.addData("Translation vector", translationVector);
                linearOpMode.telemetry.update();
            }
        } while (!targetPosition.withinRange(robotPosition, 5, 5, 5) && linearOpMode.opModeIsActive());

        update(Vector2d.ZERO, 0);
    }

    //position tracking drive method
//    public void driveToPositionOld(Position targetPosition, boolean isBlue, LinearOpMode linearOpMode) {
//        double totalXDistance = robotPosition.getAbsXDifference(targetPosition);
//        double totalYDistance = robotPosition.getAbsYDifference(targetPosition);
//        double totalHeadingDifference = robotPosition.getAbsHeadingDifference(targetPosition);
//
//        do {
//            //scale speeds based on remaining distance from target, bounded by 0 and original distance
//            //at the very beginning, x & y trans and rot will be max speed (different for each)
//            //at the end, all three speeds will be min speed (different for each)
//            //in between, the speeds will scale linearly depending on distance from target position
//            robot.updateBulkData();
//            updatePositionTracking(linearOpMode.telemetry);
//
//            Vector2d translationDirection = robotPosition.getDirectionTo(targetPosition);
//            Angle.Direction rotationDirection = robotPosition.getRotationDirectionTo(targetPosition);
//
//            double xPower = RobotUtil.scaleVal(robotPosition.getAbsXDifference(targetPosition),
//                    0, 60, 0, MAX_AUTO_DRIVE_FACTOR); //changed min to zero //was 30
//
//            double yPower = RobotUtil.scaleVal(robotPosition.getAbsYDifference(targetPosition),
//                    0, 60, 0, MAX_AUTO_DRIVE_FACTOR);
//
//            double rotationPower = RobotUtil.scaleVal(robotPosition.getAbsHeadingDifference(targetPosition),
//                    0, totalHeadingDifference, 0, MAX_AUTO_ROTATE_FACTOR);
//
//            if (robotPosition.getRotationDirectionTo(targetPosition) == Angle.Direction.CLOCKWISE) {
//                rotationPower *= -1; //todo: check sign
//            }
//
//            //todo: may need to batch normalize all three somehow (x and y should be automatically normalized)
//            Vector2d translationVector = new Vector2d(xPower * translationDirection.getX(), yPower * translationDirection.getY());
//            if (isBlue) translationVector = translationVector.reflect();
//
//            update(translationVector, rotationPower);
////kevin was here
//            if (debuggingMode) {
//                dataLogger.addField(robotPosition.x);
//                dataLogger.addField(robotPosition.y);
//                dataLogger.addField(xPower);
//                dataLogger.addField(yPower);
//                dataLogger.addField(rotationPower);
//                dataLogger.addField(translationDirection.getX());
//                dataLogger.addField(translationDirection.getY());
//                dataLogger.addField(rotationDirection.toString());
//                dataLogger.addField(translationVector.getX());
//                dataLogger.addField(translationVector.getY());
//                dataLogger.newLine();
//
//                linearOpMode.teslemetry.addData("X power", xPower);
//                linearOpMode.telemetry.addData("X difference", robotPosition.getAbsXDifference(targetPosition));
//                linearOpMode.telemetry.addData("Translation direction", translationDirection);
//                linearOpMode.telemetry.addData("Translation vector", translationVector);
//                linearOpMode.telemetry.update();
//            }
//        } while (!targetPosition.withinRange(robotPosition, 5, 5, 5) && linearOpMode.opModeIsActive());
//
//        update(Vector2d.ZERO, 0);
//    }

//    //attempt to update robot position using vuforia
//    public void updatePositionVuforia () {
//        //if a target is in view
//        Position2D vuforiaPosition = getVuforiaPosition();
//        if (vuforiaPosition != null) {
//            robotPosition.x = vuforiaPosition.x;
//            robotPosition.y = vuforiaPosition.y;
//        }
//    }


//    //Methods for moving with position tracking (Vuforia targets); UNTESTED
//    public Position2D getVuforiaPosition() {
//        TargetInfo info = vuforiaTracker.getTargetInfo();
//        return info == null ? null : new Position2D(info.xPosition, info.yPosition);
//    }

//    public Vector2d getVectorToTarget(Position2D desiredPosition) {
//        Position2D currentPosition = getCurrentPositionOnField();
//        return currentPosition == null ? null : new Vector2d(desiredPosition.x - currentPosition.x, desiredPosition.y - currentPosition.y);
//    }

//    //The following two methods are used to go to a specific position; currently untested
//    public void driveToPosition(double startx, double starty, double x, double y, double speed, long timeout, LinearOpMode linearOpMode) {
//        driveToPosition(startx, starty, new Position2D(x, y), speed, timeout, linearOpMode);
//    }

//    public void driveToPosition(double startx, double starty, Position2D position, double speed, long timeout, LinearOpMode linearOpMode) {
//        long startTime = System.currentTimeMillis();
//
//        Vector2d temp = getVectorToTarget(position);
//        Vector2d vectorToTarget = temp == null ? new Vector2d(position.x - startx, position.y - starty) : temp;
//        double distanceToTarget = vectorToTarget.getMagnitude();
//        while (distanceToTarget < ALLOWED_DISTANCE_TO_TARGET && System.currentTimeMillis() - startTime < timeout) {
//            if (distanceToTarget < START_DRIVE_SLOWDOWN_AT_CM) {
//                speed = RobotUtil.scaleVal(distanceToTarget, 0, START_DRIVE_SLOWDOWN_AT_CM, MIN_DRIVE_POWER, speed);
//                linearOpMode.telemetry.addData("speed: ", speed);
//            }
//            updateTracking(); //WAS MOVED ABOVE
//            update(vectorToTarget.normalize(Math.abs(speed)), 0); //added ABS for DEBUGGING
//
//            linearOpMode.telemetry.addData("Driving robot", "");
//            linearOpMode.telemetry.update();
//            updatePositionTracking(robot.telemetry); //update position tracking
//
//            temp = getVectorToTarget(position);
//            if (temp != null) {
//                vectorToTarget = temp;
//                distanceToTarget = vectorToTarget.getMagnitude();
//            }
//        }
//        update(Vector2d.ZERO, 0);
//        setRotateModuleMode(ROTATE_MODULES); //reset mode
//    }

    //Flips between robot and field centric
    public void setDrivingStyle(boolean toRobotCentric) {
        moduleLeft.setDrivingStyle(toRobotCentric);
        moduleRight.setDrivingStyle(toRobotCentric);
    }

    public void rotateRobot(Angle targetAngle, double power, LinearOpMode linearOpMode) {
        double startTime = System.currentTimeMillis();
        rotateModules(Vector2d.FORWARD, false, DEFAULT_TIMEOUT_ROT_MODULES, linearOpMode);

        //rotateModules
        int iterations = 0;
        boolean isNegativeRotation = robot.getRobotHeading().directionTo(targetAngle) == Angle.Direction.CLOCKWISE;

        double absHeadingDiff = robot.getRobotHeading().getDifference(targetAngle);
        while (absHeadingDiff > ALLOWED_MODULE_ROT_ERROR && linearOpMode.opModeIsActive() && iterations < MAX_ITERATIONS_ROBOT_ROTATE /*&& System.currentTimeMillis() - startTime < ROTATE_ROBOT_TIMEOUT*/) {
            absHeadingDiff = robot.getRobotHeading().getDifference(targetAngle);
            double rotMag = RobotUtil.scaleVal(absHeadingDiff, 0, 25, 0, power); //was max power 1 - WAS 0.4 max power

            if (robot.getRobotHeading().directionTo(targetAngle) == Angle.Direction.CLOCKWISE) {
                update(Vector2d.ZERO, -rotMag);
                if (!isNegativeRotation) iterations++;
            } else {
                update(Vector2d.ZERO, rotMag);
                if (isNegativeRotation) iterations++;
            }
            linearOpMode.telemetry.addData("Rotating ROBOT", "");
            linearOpMode.telemetry.update();
            updatePositionTracking(robot.telemetry); //update position tracking
        }
        update(Vector2d.ZERO, 0);
    }

    public void rotateRobot(Angle targetAngle, LinearOpMode linearOpMode) {
        rotateRobot(targetAngle, 0.4, linearOpMode);
    }


    //both modules must be within allowed error for method to return
    public void rotateModules(Vector2d direction, boolean fieldCentric, double timemoutMS, LinearOpMode linearOpMode) {
        //TODO: check if this will work with reversed modules
        double moduleLeftDifference, moduleRightDifference;
        double startTime = System.currentTimeMillis();
        do {
            moduleLeftDifference = moduleLeft.getCurrentOrientation().getDifference(direction.getAngle()); //was getRealAngle() (don't ask)
            moduleRightDifference = moduleRight.getCurrentOrientation().getDifference(direction.getAngle());
            moduleLeft.rotateModule(direction, fieldCentric);
            moduleLeft.rotateModule(direction, fieldCentric);
            moduleRight.rotateModule(direction, fieldCentric);

            linearOpMode.telemetry.addData("Rotating MODULES", "");
            linearOpMode.telemetry.addData("Top level module left difference", moduleLeftDifference);
            linearOpMode.telemetry.addData("Top level module right difference", moduleRightDifference);
            linearOpMode.telemetry.update();
            updatePositionTracking(robot.telemetry); //update position tracking
        } while ((moduleLeftDifference > ALLOWED_MODULE_ROT_ERROR || moduleRightDifference > ALLOWED_MODULE_ROT_ERROR) && linearOpMode.opModeIsActive() && System.currentTimeMillis() < startTime + timemoutMS);
        update(Vector2d.ZERO, 0);
    }


    //TRACKING METHODS
    //methods for path length tracking in autonomous (only useful for driving in straight lines)

    //new tracking method
    public void updatePositionTracking (Telemetry telemetry) {
//        if (getVuforiaPosition() != null) {
//            updatePositionVuforia();
//            return;
//        }
        Vector2d rightDisp = moduleRight.updatePositionTracking(telemetry);
        Vector2d leftDisp = moduleLeft.updatePositionTracking(telemetry);

        //todo: incorporate Vuforia data & combination logic here

        //reset heading tracking with IMU if robot is not moving
//        if (rightDisp.getMagnitude() < 0.01 && leftDisp.getMagnitude() < 0.01) {
//            //use IMU for heading instead of encoders
//            robotPosition.heading = robot.getRobotHeading();
//        } else {
            //orientation tracking with encoders
            double arcLength = moduleRight.positionChange - moduleLeft.positionChange;
            double angleChange = arcLength * 360 / 2.0 / Math.PI / WHEEL_TO_WHEEL_CM;
            robotPosition.incrementHeading(angleChange);
        //}

        rightDisp.setX(rightDisp.getX() + WHEEL_TO_WHEEL_CM /2);
        leftDisp.setX(leftDisp.getX() - WHEEL_TO_WHEEL_CM /2);

        Vector2d robotCenterDisp = new Vector2d((rightDisp.getX() + leftDisp.getX())/2, (rightDisp.getY() + leftDisp.getY())/2);
        robotCenterDisp = robotCenterDisp.rotateBy(robotPosition.heading.getAngle(Angle.AngleType.ZERO_TO_360_HEADING), Angle.Direction.CLOCKWISE); //make field centric using previous heading
        robotPosition.incrementX(robotCenterDisp.getX());
        robotPosition.incrementY(robotCenterDisp.getY());

        telemetry.addData("Robot X Position: ", robotPosition.x);
        telemetry.addData("Robot Y Position: ", robotPosition.y);
        telemetry.addData("Robot Abs Heading: ", robotPosition.heading);
    }

    public void resetDistanceTraveled() {
        previousRobotDistanceTraveled = robotDistanceTraveled;
        robotDistanceTraveled = 0;

        moduleRight.resetDistanceTraveled();
        moduleLeft.resetDistanceTraveled();

        moduleLeftLastDistance = moduleLeft.getDistanceTraveled();
        moduleRightLastDistance = moduleRight.getDistanceTraveled();
    }

    //NOTE: this is the old method (straight line tracking only)
    public void updateTracking() {
        moduleRight.updateTracking();
        moduleLeft.updateTracking();

        double moduleLeftChange = moduleLeft.getDistanceTraveled() - moduleLeftLastDistance;
        double moduleRightChange = moduleRight.getDistanceTraveled() - moduleRightLastDistance;
        robotDistanceTraveled += (moduleLeftChange + moduleRightChange) / 2;

        moduleLeftLastDistance = moduleLeft.getDistanceTraveled();
        moduleRightLastDistance = moduleRight.getDistanceTraveled();
    }

    //note: returns ABSOLUTE VALUE
    public double getDistanceTraveled() {
        return Math.abs(robotDistanceTraveled);
    }

    public void setRotateModuleMode(DriveModule.RotateModuleMode rotateModuleMode) {
        moduleLeft.rotateModuleMode = rotateModuleMode;
        moduleRight.rotateModuleMode = rotateModuleMode;
    }

    public void resetEncoders() {
        moduleRight.resetEncoders();
        moduleLeft.resetEncoders();
    }
}