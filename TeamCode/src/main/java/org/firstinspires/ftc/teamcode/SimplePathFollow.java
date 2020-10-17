package org.firstinspires.ftc.teamcode;

/*
 * Directly control both motors on both modules to move towards a given robot position and orientation
 * The path is assumed to be a linear sweep of both the position and orientation.
 * When the wheels are misaligned to travel along that path, motor power is focused on rotating the modules,
 * rather than rotating the wheel.
 * Modules are not optimized to avoid pivoting 180 degrees so reversing the robot requires pivoting the module.
 * To avoid module oscillations, the modules favor the previous pivot direction unless the module is within
 * 120 degrees of the target position
 */
public class SimplePathFollow {

    SimpleTracking.Vec2D targetTranslation;
    SimpleTracking.Vec2D targetRotation;
    SimpleTracking.Vec2D targetLeftPower, targetRightPower; // target power vectors for each module

    double leftTopPower, leftBottomPower, rightTopPower, rightBottomPower;

    public static final double PIVOT_THRESHOLD = 25;

    public SimplePathFollow() {
        targetTranslation = new SimpleTracking.Vec2D();
        targetRotation = new SimpleTracking.Vec2D();

        targetLeftPower = new SimpleTracking.Vec2D();
        targetRightPower = new SimpleTracking.Vec2D();
    }

    /*
     * Use the current state of the robot in the given tracking variable to update the robot's motor powers.
     * Modules will be aligned to move along the path from the current position/ orientation to the requested
     * position/ orientation.  If the wheel modules are misaligned, motor power will be shifted to rotating the
     * module.  As the modules become more aligned, motor power will switch to rotating the wheel.
     * The return value is the distance to the target
     */
    public double moveToTarget(Robot robot, SimpleTracking tracking, double x, double y, double orientation, double speed) {

        // calculate the desired module orientation
        // The translation vector is the target location minus the current location
        // These lcoations are in field oriented coordinates so rotate them by the robot's orientation
        // to get the robot-centric translation vector
        targetTranslation.set(x - tracking.lastRobotX, y - tracking.lastRobotY);
        targetTranslation.rotateByDegrees(-tracking.lastRobotOrientation);
        double deltaOrientation = orientation - tracking.lastRobotOrientation;
        // remap deltaOrientation to {-180, 180}
        deltaOrientation %= 360;
        if (deltaOrientation < -180) deltaOrientation += 360;
        if (deltaOrientation >= 180) deltaOrientation -= 360;

        targetRotation.setMagnitudeDirectionDegrees(Math.abs(deltaOrientation/360) * SimpleTracking.WHEEL_BASE * Math.PI, (deltaOrientation >= 0) ? 0 : 180);

        // need a deadband to avoid robot orientation oscillations

        targetLeftPower.add(targetTranslation, targetRotation);
        targetRightPower.subtract(targetTranslation, targetRotation);
        // find acaling factor based on the magnitude of each vector and the relative speed requested
        double scale = targetLeftPower.getMagnitude();
        double tmp = targetRightPower.getMagnitude();
        if (tmp > scale) scale = tmp;
        scale /= speed;
        targetLeftPower.scale(1/scale);
        targetRightPower.scale(1/scale);

        double leftPivotError = targetLeftPower.getAngleDegrees() - tracking.lastLeftOrientation;
        if (leftPivotError <= -180) leftPivotError += 360;
        if (leftPivotError > 180) leftPivotError -= 360;
        double rightPivotError = targetRightPower.getAngleDegrees() - tracking.lastRightOrientation;
        if (rightPivotError <= -180) rightPivotError += 360;
        if (rightPivotError > 180) rightPivotError -= 360;

        double leftScaling = 1 - Math.abs(leftPivotError) / 180 * 8;
        if (leftScaling < -1) leftScaling = -1;

        leftTopPower = - targetLeftPower.getMagnitude();
        leftBottomPower = - leftTopPower; // start with full wheel drive

        rightTopPower = -targetRightPower.getMagnitude();
        rightBottomPower = - rightTopPower; // start with full wheel drive
        double rightScaling = 1 - Math.abs(rightPivotError) / 180 * 8;
        if (rightScaling < -1) rightScaling = -1;

        // coordinate the alignment of both modules - if one is realigning, then cut power to the other and make sure the misaligned
        // module gets full rotation powers
        if ((Math.abs(leftPivotError) > PIVOT_THRESHOLD) && (Math.abs(rightPivotError) < PIVOT_THRESHOLD)) {
            rightTopPower = rightBottomPower = 0; // stop the right side
            leftTopPower = -0.7;
            leftBottomPower = 0.7;
            leftScaling = -1;
        } else if ((Math.abs(leftPivotError) < PIVOT_THRESHOLD) && (Math.abs(rightPivotError) > PIVOT_THRESHOLD)) {
            leftTopPower = leftBottomPower = 0;
            rightTopPower = -0.7;
            rightBottomPower = 0.7;
            rightScaling = -1;
        }

        if (leftPivotError > 0) {
            leftTopPower *= leftScaling;
        } else {
            leftBottomPower *= leftScaling;
        }

        if (rightPivotError > 0) {
            rightTopPower *= rightScaling;
        } else {
            rightBottomPower *= rightScaling;
        }

        robot.driveController.moduleLeft.motor1.setPower(leftTopPower);
        robot.driveController.moduleLeft.motor2.setPower(leftBottomPower);
        robot.driveController.moduleRight.motor1.setPower(rightTopPower);
        robot.driveController.moduleRight.motor2.setPower(rightBottomPower);

        return targetTranslation.getMagnitude() + targetRotation.getMagnitude(); // rough estimate of how far robot is from its target
    }

    public void stop(Robot robot) {
        robot.driveController.moduleLeft.motor1.setPower(0);
        robot.driveController.moduleLeft.motor2.setPower(0);
        robot.driveController.moduleRight.motor1.setPower(0);
        robot.driveController.moduleRight.motor2.setPower(0);
    }
}
