package org.firstinspires.ftc.teamcode;

import static java.lang.Math.*;

public class AutoFunctions {

    // method to drive in a certain direction (facing forward, but sliding towards a specific angle)
    public static void driveTowards(double speedVal, double relativeAngleToPoint, UpliftRobot robot) {
        double lfPow = sin(toRadians(90 - relativeAngleToPoint) + (0.25 * PI)) * speedVal;
        double rfPow = sin(toRadians(90 - relativeAngleToPoint) - (0.25 * PI)) * speedVal;
        double lbPow = sin(toRadians(90 - relativeAngleToPoint) - (0.25 * PI)) * speedVal;
        double rbPow = sin(toRadians(90 - relativeAngleToPoint) + (0.25 * PI)) * speedVal;

        // find max total input out of the 4 motors
        double maxVal = abs(lfPow);
        if(abs(rfPow) > maxVal) {
            maxVal = abs(rfPow);
        }
        if(abs(lbPow) > maxVal) {
            maxVal = abs(lbPow);
        }
        if(abs(rbPow) > maxVal) {
            maxVal = abs(rbPow);
        }

        if(maxVal < (1 / sqrt(2))) {
            maxVal = 1 / sqrt(2);
        }

        // set the scaled powers
        robot.leftFront.setPower(lfPow / maxVal);
        robot.rightFront.setPower(rfPow / maxVal);
        robot.leftBack.setPower(lbPow / maxVal);
        robot.rightBack.setPower(rbPow / maxVal);
    }

    // method to drive to a target point, within specific tolerance
    public static void driveToPosition(double x, double y, double speedVal, double tolerance, UpliftRobot robot) {

    }

    // method to pass through a target point (within a large tolerance) and continue without stopping
    public static void passThroughPosition(double x, double y, double speedVal, UpliftRobot robot) {

    }

    // method to constantly spin ( [+] for clockwise and [-] for counter-clockwise )
    public static void spin(double speed, UpliftRobot robot) {

    }

    // method to turn a certain number of degrees ( [+] for clockwise and [-] for counter-clockwise )
    public static void turn(double degrees, double speed, double tolerance, UpliftRobot robot) {

    }

    // method to turn TO a certain angle (within the angle restrictions), with either the shortest path (technique 0) or through a specified direction in the direction indicator (clockwise for 1, counter-clockwise for 2)
    public static void turnTo(double targetAngle, double speed, double tolerance, int directionIndex, UpliftRobot robot) {

    }

    // method to stop all of the drive motors
    public static void stopMotors(UpliftRobot robot) {

    }

    // method to shoot three rings
    public static void autoHighGoalShoot(UpliftRobot robot) {

    }
}
