package org.firstinspires.ftc.teamcode.tracking;

/*
 * Basic tracking of robot position and orientation
 */

import org.firstinspires.ftc.teamcode.drivecontrol.Angle;
import org.firstinspires.ftc.teamcode.drivecontrol.Robot;

public class SimpleTracking {

    /*
     * This module calculates the orientation of each wheel module and the orientation of the robot
     * each cycle.  These variables hold on to the most recent values so the update routine can use
     * the average of the current value and past value to update the position
     */
    double lastLeftOrientation, lastRightOrientation;

    double lastleftDistance, lastRightDistance;

    Vec2D leftModuleVector, rightModuleVector;

    // hold on to the change in each module so we can use past pivot direction to guide the future direction
    double leftPivotChange, rightPivotChange;

    double lastRobotX, lastRobotY;
    double lastRobotOrientation;

    Vec2D robotPositionChange, robotOrientationChange; // current cycle's incremental change in position and orientation

    double leftTopMotorPosition;
    double leftBottomMotorPosition;
    double rightTopMotorPosition;
    double rightBottomMotorPosition;

    /*
     * Constants used to calculate module and robot position and orientation.  These should be moved
     * to the robot constants class to avoid duplication with DriveModule
     */
    private static final double TICKS_PER_MODULE_REV = 1014;//10 * (double)(60)/14 * (double)(48)/15 * (double)(82)/22; //ticks per MODULE revolution

    private static final double TICKS_PER_WHEEL_REV = 10 * (double)(60)/14 * (double)(48)/15 * (double)(82)/22 * (double)(14)/60; //ticks per WHEEL revolution

    private static final double CM_WHEEL_DIAMETER = 2.5 * 2.54;
    private static final double CM_PER_WHEEL_REV = CM_WHEEL_DIAMETER * Math.PI;

    public static final double WHEEL_BASE = 14*2.56; // distance between the wheel modules


    /*
     * Observed encoder changes:
     * Left module wheel rotates forward, top encoder decreases, bottom encoder increases
     * Right module wheel rotates forward, top encoder decreases, bottom encoder increases
     * Left module pivot clockwise (viewed on top), both encoders increase
     * Right module pivot clockwise (viewed on top), both encoders increase
     */

    public SimpleTracking() {
        lastLeftOrientation = 0;
        lastRightOrientation = 0;
        lastleftDistance = 0;
        lastRightDistance = 0;

        lastRobotX = 0;
        lastRobotY = 0;
        lastRobotOrientation = 0;

        leftModuleVector = new Vec2D();
        rightModuleVector = new Vec2D();

        robotPositionChange = new Vec2D();
        robotOrientationChange = new Vec2D();
    }

    public void setPosition(double x, double y) {
        lastRobotX = x;
        lastRobotY = y;
    }

    public void setOrientationDegrees(double orientation) {
        lastRobotOrientation = orientation;
    }

    // this is just for testing
    public void setModuleOrientation(Robot robot) {
        leftTopMotorPosition = robot.bulkData1.getMotorCurrentPosition(robot.driveController.moduleLeft.motor1);
        leftBottomMotorPosition = robot.bulkData1.getMotorCurrentPosition(robot.driveController.moduleLeft.motor2);
        rightTopMotorPosition = robot.bulkData1.getMotorCurrentPosition(robot.driveController.moduleRight.motor1);
        rightBottomMotorPosition = robot.bulkData1.getMotorCurrentPosition(robot.driveController.moduleRight.motor2);

        //calculate the current orientations of the wheel modules in degrees, 0 is forward, positive clockwise viewed from the top
        double leftOrientation = (((leftTopMotorPosition + leftBottomMotorPosition) / (2 * TICKS_PER_MODULE_REV))
                - Math.floor((leftTopMotorPosition + leftBottomMotorPosition) / (2 * TICKS_PER_MODULE_REV))) * 360;
        double rightOrientation = (((rightTopMotorPosition + rightBottomMotorPosition) / (2 * TICKS_PER_MODULE_REV))
                - Math.floor((rightTopMotorPosition + rightBottomMotorPosition) / (2 * TICKS_PER_MODULE_REV))) * 360;
        double leftDistance = ((leftBottomMotorPosition - leftTopMotorPosition) / (2 * TICKS_PER_WHEEL_REV / CM_PER_WHEEL_REV));
        double rightDistance = ((rightBottomMotorPosition - rightTopMotorPosition) / (2 * TICKS_PER_WHEEL_REV / CM_PER_WHEEL_REV));

        lastLeftOrientation = remapDegrees(leftOrientation);
        lastRightOrientation = remapDegrees(rightOrientation);
        lastleftDistance = leftDistance;
        lastRightDistance = rightDistance;

    }

    public void updatePosition(Robot robot, boolean useIMU, boolean isBlue) {

        // collect the current motor encoder values
        leftTopMotorPosition = robot.bulkData1.getMotorCurrentPosition(robot.driveController.moduleLeft.motor1);
        leftBottomMotorPosition = robot.bulkData1.getMotorCurrentPosition(robot.driveController.moduleLeft.motor2);
        rightTopMotorPosition = robot.bulkData1.getMotorCurrentPosition(robot.driveController.moduleRight.motor1);
        rightBottomMotorPosition = robot.bulkData1.getMotorCurrentPosition(robot.driveController.moduleRight.motor2);

        //calculate the current orientations of the wheel modules in degrees, 0 is forward, positive clockwise viewed from the top
        double leftOrientation = (((leftTopMotorPosition + leftBottomMotorPosition) / (2 * TICKS_PER_MODULE_REV))
                - Math.floor((leftTopMotorPosition + leftBottomMotorPosition) / (2 * TICKS_PER_MODULE_REV))) * 360;
        double rightOrientation = (((rightTopMotorPosition + rightBottomMotorPosition) / (2 * TICKS_PER_MODULE_REV))
                - Math.floor((rightTopMotorPosition + rightBottomMotorPosition) / (2 * TICKS_PER_MODULE_REV))) * 360;

        /*
         * Calculate the current rotation distance of the wheel modules.  This is the total rotation distance the wheel
         * has travelled since the last reset.
         */
        double leftDistance = ((leftBottomMotorPosition - leftTopMotorPosition) / (2 * TICKS_PER_WHEEL_REV / CM_PER_WHEEL_REV));
        double rightDistance = ((rightBottomMotorPosition - rightTopMotorPosition) / (2 * TICKS_PER_WHEEL_REV / CM_PER_WHEEL_REV));

        leftModuleVector.setMagnitudeDirectionDegrees(leftDistance - lastleftDistance, getAverageOrientation(leftOrientation, lastLeftOrientation));
        rightModuleVector.setMagnitudeDirectionDegrees(rightDistance - lastRightDistance, getAverageOrientation(rightOrientation, lastRightOrientation));

        /*
         * Now update the robot position and orientation.
         * The robot's position change (translation vector) is the average of the left and right modules' vectors
         * The robot's orientation change (rotation vector) is half the difference of the left and right modules' vectors
         */
        robotPositionChange.add(leftModuleVector, rightModuleVector).scale(0.5);
        robotOrientationChange.subtract(leftModuleVector, rightModuleVector).scale(0.5);


        /*
         * Update the robot's field-centric position and orientation.  Assume that the robot travels along
         * a straight line from the last position.  The distance of the travel is the magnitude of the translation vector.
         * The direction of the travel is the average of the previous and current robot orientations
         */
        double rotationDirection = ((robotOrientationChange.getAngleDegrees() > 90) && (robotOrientationChange.getAngleDegrees() <= 270)) ? -1 : 1;
        double robotOrientation = lastRobotOrientation + rotationDirection * 360 * (robotOrientationChange.getMagnitude() / (WHEEL_BASE * Math.PI));

        //added 1-20
        if (useIMU) {
            if (isBlue) robotOrientation = robot.getRobotHeading().rotateBy(180).getAngle(Angle.AngleType.ZERO_TO_360_HEADING);
            else robotOrientation = robot.getRobotHeading().getAngle(Angle.AngleType.ZERO_TO_360_HEADING);
        }

        double averageOrientation = getAverageOrientation(lastRobotOrientation, robotOrientation);
        double positionChangeAngle = robotPositionChange.getAngleDegrees();
        double positionChangeMagnitude = robotPositionChange.getMagnitude();

        lastRobotX += positionChangeMagnitude * Math.cos(Vec2D.degreesToRads(averageOrientation + positionChangeAngle));
        lastRobotY += positionChangeMagnitude * Math.sin(Vec2D.degreesToRads(averageOrientation + positionChangeAngle));

        leftPivotChange = lastLeftOrientation - leftOrientation;
        rightPivotChange = lastRightOrientation - rightOrientation;

        // Save the current module values for next time, ensuring that the orientation are in the range [0,360)
        lastLeftOrientation = remapDegrees(leftOrientation);
        lastRightOrientation = remapDegrees(rightOrientation);
        lastleftDistance = leftDistance;
        lastRightDistance = rightDistance;

        lastRobotOrientation = remapDegrees(robotOrientation);
    }

    public void updatePosition(Robot robot) {
        updatePosition(robot, false, false); //isblue is irrelavant without useIMU
    }


        private double remapDegrees(double degrees) {
        if (degrees < 0) degrees += 360;
        if (degrees >= 360) degrees -= 360;
        return degrees;
    }

    // this is used for the incremental orientation changes.  Assume that a change larger than 180 is not possible
    private double getAverageOrientation(double o1, double o2) {
        double average = (o1 + o2) * 0.5;
        if (o1 - o2 > 180) average -= 180;
        if (o2 - o1 > 180) average -= 180;
        if (average < 0) average += 360;
        return average;
    }

    public static class Vec2D {
        double x;
        double y;

        public Vec2D(double x, double y) {
            this.x = x;
            this.y = y;
        }

        public Vec2D() {
            this(0, 0);
        }

        public void set (double x, double y) {
            this.x = x;
            this.y = y;
        }

        // Set the vector using the given magnitude and direction.  The direction is given in degrees
        public void setMagnitudeDirectionDegrees(double mag, double dir) {
            double rads = (90-dir) * Math.PI / 180;
            x = mag * Math.cos(rads);
            y = mag * Math.sin(rads);
        }

        public double getAngleDegrees() {
            double angle = 90 - (Math.atan2(y, x) * 180 / Math.PI);
            if (angle < 0) angle += 360;
            return angle;
        }

        public Vec2D add(Vec2D addend1, Vec2D addend2) {
            this.x = addend1.x + addend2.x;
            this.y = addend1.y + addend2.y;
            return this;
        }

        public Vec2D subtract(Vec2D addend1, Vec2D addend2) {
            this.x = addend1.x - addend2.x;
            this.y = addend1.y - addend2.y;
            return this;
        }

        public Vec2D scale(double scale) {
            this.x *= scale;
            this.y *= scale;
            return this;
        }

        public double getMagnitude() {
            return Math.hypot(x, y);
        }

        public static double radsToDegress(double rads) {
            return (-rads * 180 / Math.PI) + 90;
        }

        public static double degreesToRads(double degrees) {
            return (90-degrees) * Math.PI / 180;
        }

        public Vec2D rotateByDegrees(double degrees) {
            // note that there is no phase shift when converting degrees to radians for the rotation matrix
            double rads = (-degrees) * Math.PI / 180;
            double newX = x * Math.cos(rads) - y * Math.sin(rads);
            double newY = x * Math.sin(rads) + y * Math.cos(rads);
            x = newX;
            y = newY;

            return this;
        }
    }
}
