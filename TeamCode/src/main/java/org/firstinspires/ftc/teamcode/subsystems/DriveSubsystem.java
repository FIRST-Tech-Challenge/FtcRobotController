package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.UpliftRobot;
import org.firstinspires.ftc.teamcode.toolkit.misc.MathFunctions;
import org.firstinspires.ftc.teamcode.toolkit.misc.PathPoint;
import org.firstinspires.ftc.teamcode.toolkit.core.Subsystem;
import org.firstinspires.ftc.teamcode.toolkit.misc.Point;

import java.util.ArrayList;

import static java.lang.Math.*;

public class DriveSubsystem extends Subsystem {

    public UpliftRobot robot;
    LinearOpMode opMode;
    private DcMotor leftFront, leftBack, rightFront, rightBack;

    // shooting positions
    public static Point highGoalShootingPt = new Point(106, 70, 0);
    public static Point powershotShootingPt1 = new Point(67.5, 70,0);
    public static Point powershotShootingPt2 = new Point(74.5, 70,0);
    public static Point powershotShootingPt3 = new Point(81.5, 70,0);


    // direction constants
    public static final int CLOCKWISE = 1;
    public static final int COUNTER_CLOCKWISE = 2;
    public static final int QUICKEST_DIRECTION = 0;

    public DriveSubsystem(UpliftRobot robot){
        super(robot);
        this.robot = robot;
        this.opMode = robot.opMode;
        this.leftFront = robot.leftFront;
        this.leftBack = robot.leftBack;
        this.rightFront = robot.rightFront;
        this.rightBack = robot.rightBack;
    }

    @Override
    public void enable() {

    }

    @Override
    public void disable() {
        stopMotors();
    }

    @Override
    public void stop() {

    }

    @Override
    public void safeDisable() {
        stopMotors();
    }

    public void driveToPosition(double xPosition, double yPosition, double movementSpeed, double tolerance, double targetAngle, int turnDirection) {
        double xDistanceToPoint = xPosition - robot.worldX;
        double yDistanceToPoint = yPosition - robot.worldY;
        double distanceToPoint = hypot(xDistanceToPoint, yDistanceToPoint);

        // use this to see how quickly to turn while driving to point
        double initialDistanceToPoint = distanceToPoint;
        double relativeAngle = toDegrees(MathFunctions.atan2UL(yDistanceToPoint, xDistanceToPoint)) - robot.worldAngle;

        double approachZone = 20;

        while (abs(distanceToPoint) > tolerance) {
            if (robot.driverCancel || !opMode.opModeIsActive() || opMode.isStopRequested()) {
                // breakaway statement for teleop
                disable();
                return;
            }

            driveTowards(MathFunctions.slowApproach(movementSpeed, distanceToPoint, approachZone, tolerance), relativeAngle, targetAngle, initialDistanceToPoint, turnDirection);

            xDistanceToPoint = xPosition - robot.worldX;
            yDistanceToPoint = yPosition - robot.worldY;
            distanceToPoint = hypot(xDistanceToPoint, yDistanceToPoint);
            relativeAngle = toDegrees(MathFunctions.atan2UL(yDistanceToPoint, xDistanceToPoint)) - robot.worldAngle;
        }

        // arrived at point, so stop
        stopMotors();

        // correct angle to be preferred angle
        turnTo(targetAngle, DriveSubsystem.QUICKEST_DIRECTION);

    }

    public void driveToPosition(double xPosition, double yPosition, double movementSpeed, double targetAngle, int turnDirection) {
        driveToPosition(xPosition, yPosition, movementSpeed, 2, targetAngle, turnDirection);
    }

    public void driveToPosition(double xPosition, double yPosition, double movementSpeed, double targetAngle) {
        driveToPosition(xPosition, yPosition, movementSpeed, 2, targetAngle, DriveSubsystem.QUICKEST_DIRECTION);
    }

    public void passThroughPosition(double xPosition, double yPosition, double movementSpeed, double targetAngle, int turnDirection) {
        double xDistanceToPoint = xPosition - robot.worldX;
        double yDistanceToPoint = yPosition - robot.worldY;
        double distanceToPoint = hypot(xDistanceToPoint, yDistanceToPoint);

        // use this to see how quickly to turn while driving to point
        double initialDistanceToPoint = distanceToPoint;

        double relativeAngle = toDegrees(MathFunctions.atan2UL(yDistanceToPoint, xDistanceToPoint)) - robot.worldAngle;
        while (distanceToPoint > 4) {
            if (robot.driverCancel || !opMode.opModeIsActive() || opMode.isStopRequested()) {
                // breakaway statement for teleop
                disable();
                return;
            }

            driveTowards(movementSpeed, relativeAngle, targetAngle, initialDistanceToPoint, turnDirection);

            xDistanceToPoint = xPosition - robot.worldX;
            yDistanceToPoint = yPosition - robot.worldY;
            distanceToPoint = hypot(xDistanceToPoint, yDistanceToPoint);
            relativeAngle = toDegrees(MathFunctions.atan2UL(yDistanceToPoint, xDistanceToPoint)) - robot.worldAngle;
        }

    }

    public void passThroughPosition(double xPosition, double yPosition, double movementSpeed, double targetAngle) {
        passThroughPosition(xPosition, yPosition, movementSpeed, targetAngle, DriveSubsystem.QUICKEST_DIRECTION);
    }

    public void driveTowards(double speedVal, double relativeAngleToPoint, double targetAngle, double initialDistToPt, int turnDirection) {
        double turnVal = 0;
        double initialAngle = robot.worldAngle;
        double turnAngle = MathFunctions.angleRestrictions(targetAngle - initialAngle);

        if(turnAngle > 30) {
            if(turnDirection == CLOCKWISE) {
                turnVal = Range.clip((60 / initialDistToPt) * (180 / Math.abs(turnAngle)), -1, 1);
            } else if(turnDirection == COUNTER_CLOCKWISE) {
                turnVal = -Range.clip((60 / initialDistToPt) * (180 / Math.abs(turnAngle)), -1, 1);
            } else {
                turnVal = Range.clip((60 / initialDistToPt) * (180 / Math.abs(turnAngle)), -1, 1);
            }
        } else if(turnAngle > 5) {
            turnVal = 0.2;
        } else if(turnAngle < -30) {
            if(turnDirection == CLOCKWISE) {
                turnVal = Range.clip(60 / initialDistToPt * (180 / Math.abs(turnAngle)), -1, 1);
            } else if(turnDirection == COUNTER_CLOCKWISE) {
                turnVal = -Range.clip(60 / initialDistToPt * (180 / Math.abs(turnAngle)), -1, 1);
            } else {
                turnVal = Range.clip(60 / initialDistToPt * (180 / Math.abs(turnAngle)), -1, 1);
            }
        } else if(turnAngle < -5) {
            turnVal = -0.2;
        } else {
            turnVal = 0;
        }

        double x = 1.4 * speedVal * sin(toRadians(relativeAngleToPoint));
        double y = speedVal * cos(toRadians(relativeAngleToPoint));

        double lf = y + x + turnVal;
        double rf = y - x - turnVal;
        double lb = y - x + turnVal;
        double rb = y + x - turnVal;

        // find max total input out of the 4 motors
        double maxVal = abs(lf);
        if(abs(rf) > maxVal){
            maxVal = abs(rf);
        }
        if(abs(lb) > maxVal){
            maxVal = abs(lb);
        }
        if(abs(rb) > maxVal){
            maxVal = abs(rb);
        }

        if(maxVal < (1 / sqrt(2))) {
            maxVal = 1 / sqrt(2);
        }

        // set the scaled powers
        leftFront.setPower(lf / maxVal);
        rightFront.setPower(rf / maxVal);
        leftBack.setPower(lb / maxVal);
        rightBack.setPower(rb / maxVal);
    }

    // method to move a certain direction at a given speed
    public void teleDrive(double x, double y, double turnVal) {

        double lf = y + x + turnVal;
        double rf = y - x - turnVal;
        double lb = y - x + turnVal;
        double rb = y + x - turnVal;

        // find max total input out of the 4 motors, if none above 1/sqrt(2) , then max is 1/sqrt(2)
        double maxVal = 1 / sqrt(2);
        if(abs(lf) > maxVal){
            maxVal = abs(lf);
        }
        if(abs(rf) > maxVal){
            maxVal = abs(rf);
        }
        if(abs(lb) > maxVal){
            maxVal = abs(lb);
        }
        if(abs(rb) > maxVal){
            maxVal = abs(rb);
        }

        // set the scaled powers
        leftFront.setPower(lf / maxVal);
        rightFront.setPower(rf / maxVal);
        leftBack.setPower(lb / maxVal);
        rightBack.setPower(rb / maxVal);
    }

    public void followPath(ArrayList<PathPoint> path) {
        // tell the robot to map out the path and follow it
        for (PathPoint pt : path) {
            driveToPosition(pt.x, pt.y, pt.moveSpeed, 0, 0);
            if(robot.driverCancel) {
                safeDisable();
                return;
            }
        }
    }

    // method to drive forwards
    public void moveForward(double speed) {
        leftFront.setPower(speed);
        rightFront.setPower(speed);
        leftBack.setPower(speed);
        rightBack.setPower(speed);
    }

    // method to constantly spin ( [+] for clockwise and [-] for counter-clockwise )
    public void spin(double speed) {
        speed = Range.clip(speed, -1, 1);
        leftFront.setPower(speed);
        leftBack.setPower(speed);
        rightFront.setPower(-speed);
        rightBack.setPower(-speed);
    }

    // method to turn a certain number of degrees ( [+] for clockwise and [-] for counter-clockwise )
    public void turn(double degrees) {
        double initialAngle = robot.rawAngle;
        double targetAngle = initialAngle + degrees;
        double power = 0;
        double angleRemaining = targetAngle - robot.rawAngle;

        double initialTime = System.currentTimeMillis();
        while ((angleRemaining < -2 || angleRemaining > 2) && opMode.opModeIsActive() && System.currentTimeMillis() - initialAngle < 3500) {
            angleRemaining = targetAngle - robot.rawAngle;
            if(angleRemaining > 90) {
                power = 1;
//            } else if(angleRemaining > 45) {
//                power = 0.5;
//            } else if (angleRemaining > 30) {
//                power = 0.3;
//            } else if(angleRemaining > 5) {
//                power = 0.2;
            } else if(angleRemaining > 0) {
                power = Math.pow(Math.abs(angleRemaining) / 120, 0.6);
            } else if(angleRemaining < -90) {
                power = -1;
//            } else if(angleRemaining < -45) {
//                power = -0.5;
//            } else if(angleRemaining < -30) {
//                power = -0.3;
//            } else if(angleRemaining < -5) {
//                power = -0.2;
            } else if(angleRemaining < 0) {
                power = -Math.pow(Math.abs(angleRemaining) / 120, 0.6);
            }
            Log.i("Odometry", "turning power: " + power);
            Log.i("Odometry", "Angle remaining;" + angleRemaining);
            spin(power);
            angleRemaining = targetAngle - robot.rawAngle;
        }

        stopMotors();

    }

    // method to turn TO a certain angle (within the angle restrictions), with either the shortest path (technique 0) or through a specified direction in the direction indicator (clockwise for 1, counter-clockwise for 2)
    public void turnTo(double targetAngle, int directionIndex) {
        double initialAngle = robot.worldAngle;
        double quickestTurnAngle = MathFunctions.angleRestrictions(targetAngle - initialAngle);
        if(quickestTurnAngle > 0) {
            if(directionIndex == CLOCKWISE) {
                turn(quickestTurnAngle);
            } else if(directionIndex == COUNTER_CLOCKWISE) {
                turn(quickestTurnAngle - 360);
            } else {
                turn(quickestTurnAngle);
            }
        } else if(quickestTurnAngle < 0) {
            if(directionIndex == CLOCKWISE) {
                turn(quickestTurnAngle + 360);
            } else if(directionIndex == COUNTER_CLOCKWISE) {
                turn(quickestTurnAngle);
            } else {
                turn(quickestTurnAngle);
            }
        } else {
            // NOTHING...
        }

    }

    // method to stop all of the drive motors
    public void stopMotors() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }
    
}
