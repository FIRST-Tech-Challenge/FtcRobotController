package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.toolkit.core.UpliftAuto;
import org.firstinspires.ftc.teamcode.toolkit.misc.UpliftMath;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.pow;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;
import static java.lang.Math.toRadians;

@Autonomous(name = "Auto", group = "OpModes")
public class Auto extends UpliftAuto {

    UpliftRobot robot;
    DcMotor lf;
    DcMotor rf;
    DcMotor lb;
    DcMotor rb;


    public static final int CLOCKWISE = 1, COUNTER_CLOCKWISE = 2, QUICKEST_DIRECTION = 0;

    @Override
    public void initHardware() {
        robot = new UpliftRobot(this);
        lf = robot.leftFront;
        rf = robot.rightFront;
        lb = robot.leftBack;
        rb = robot.rightBack;
    }

    @Override
    public void initAction() {

    }

    @Override
    public void body() throws InterruptedException {

    }

    @Override
    public void exit() throws InterruptedException {
        robot.writePositionToFiles();
        robot.stopThreads();
    }

    // method to drive in a certain direction (facing forward, but sliding towards a specific angle)
    public void driveTowards(double speedVal, double relativeAngleToPoint) {
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
        lf.setPower(lfPow / maxVal);
        rf.setPower(rfPow / maxVal);
        lb.setPower(lbPow / maxVal);
        rb.setPower(rbPow / maxVal);
    }

    // method to drive to a target point, within specific tolerance
    public void driveToPosition(double x, double y, double speedVal, double tolerance) {
        double xDist = x - robot.worldX;
        double yDist = y - robot.worldY;
        double distance = sqrt(pow(xDist, 2) + pow(yDist,2));
        double angle = UpliftMath.atan2UL(yDist,xDist);
        while(distance > tolerance) {
            driveTowards(speedVal, angle);
            xDist = x - robot.worldX;
            yDist = y - robot.worldY;
            distance = sqrt(pow(xDist, 2) + pow(yDist,2));
            angle = UpliftMath.atan2UL(yDist,xDist);
        }
    }

    // method to pass through a target point (within a large tolerance) and continue without stopping
    public void passThroughPosition(double x, double y, double speedVal) {

    }

    // method to constantly spin ( [+] for clockwise and [-] for counter-clockwise )
    public void spin(double speed) {
        speed = Range.clip(speed, -1, 1);
        lf.setPower(speed);
        lb.setPower(speed);
        rf.setPower(-speed);
        rb.setPower(-speed);
    }

    // method to turn a certain number of degrees ( [+] for clockwise and [-] for counter-clockwise )
    public void turn(double degrees, double speed, double tolerance) {
        double initialAngle = robot.rawAngle;
        double targetAngle = initialAngle + degrees;
        // if turning counter-clockwise
        while(Math.abs(degrees) > tolerance) {
            if(degrees > 30) {
                spin(speed);
            } else if(degrees > 10) {
                spin(0.4);
            } else if(degrees > 5) {
                spin(0.2);
            } else if(degrees > 0) {
                spin(0.15);
            } else if(degrees < -30) {
                spin(-speed);
            } else if(degrees < -10) {
                spin(-0.4);
            } else if(degrees < -5) {
                spin(-0.2);
            } else if(degrees < 0) {
                spin(-0.15);
            } else {
                stopMotors();
                return;
            }
            degrees = targetAngle - robot.rawAngle;
        }
        stopMotors();
    }

    // method to turn TO a certain angle (within the angle restrictions), with either the shortest path (technique 0) or through a specified direction in the direction indicator (clockwise for 1, counter-clockwise for 2)
    public void turnTo(double targetAngle, double speed, double tolerance, int directionIndex) {
        double initialAngle = robot.worldAngle;
        double quickestTurnAngle = UpliftMath.angleRestrictions(targetAngle - initialAngle);
        if(quickestTurnAngle > 0) {
            if(directionIndex == CLOCKWISE) {
                turn(quickestTurnAngle, speed, tolerance);
            } else if(directionIndex == COUNTER_CLOCKWISE) {
                turn(quickestTurnAngle - 360, speed, tolerance);
            } else {
                turn(quickestTurnAngle, speed, tolerance);
            }
        } else if(quickestTurnAngle < 0) {
            if(directionIndex == CLOCKWISE) {
                turn(quickestTurnAngle + 360, speed, tolerance);
            } else if(directionIndex == COUNTER_CLOCKWISE) {
                turn(quickestTurnAngle, speed, tolerance);
            } else {
                turn(quickestTurnAngle, speed, tolerance);
            }
        } else {
            // NOTHING
        }
    }

    // method to stop all of the drive motors
    public void stopMotors() {
        lf.setPower(0);
        rf.setPower(0);
        lb.setPower(0);
        rb.setPower(0);
    }
}
