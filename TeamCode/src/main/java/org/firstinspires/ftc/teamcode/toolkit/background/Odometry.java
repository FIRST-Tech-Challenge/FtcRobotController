package org.firstinspires.ftc.teamcode.toolkit.background;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.UpliftRobot;
import org.firstinspires.ftc.teamcode.toolkit.core.Background;
import org.firstinspires.ftc.teamcode.toolkit.misc.MathFunctions;

public class Odometry extends Background {

    LinearOpMode opMode;
    UpliftRobot robot;
    private DcMotor leftEncoder, rightEncoder, centerEncoder;

    private double finalLeftDistance, finalRightDistance, finalCenterDistance;
    private double initialLeftDistance, initialRightDistance, initialCenterDistance;

    public Odometry(UpliftRobot robot) {
        super(robot);
        this.robot = robot;
        this.opMode = robot.opMode;
        this.leftEncoder = robot.leftFront;
        this.rightEncoder = robot.rightFront;
        this.centerEncoder = robot.rightBack;
    }

    @Override
    public void loop() {
        updatePosition();
//        Log.i("Odometry:", "X: " + robot.worldX + "   Y: " + robot.worldY + "   Angle: " + robot.worldAngle);
//        Log.i("Lf", robot.leftFront.getPower() + "");
//        Log.i("Rf", robot.rightFront.getPower() + "");
//        Log.i("Lb", robot.leftBack.getPower() + "");
//        Log.i("Rb", robot.rightBack.getPower() + "");

    }

    // method to update the current position and angle of the robot (relative to left-rear edge of robot and left-rear field corner)
    public void updatePosition() {

        finalLeftDistance = (getLeftTicks() / UpliftRobot.COUNTS_PER_INCH);
        finalRightDistance = (getRightTicks() / UpliftRobot.COUNTS_PER_INCH);
        finalCenterDistance = (getCenterTicks() / UpliftRobot.COUNTS_PER_INCH);

        double deltaLeftDistance = finalLeftDistance - initialLeftDistance;
        double deltaRightDistance = finalRightDistance - initialRightDistance;
        double deltaCenterDistance = finalCenterDistance - initialCenterDistance;


        double changeInRobotOrientation = Math.toDegrees((deltaLeftDistance - deltaRightDistance) / (UpliftRobot.robotEncoderWheelDistance));
        double deltaHorizontal = deltaCenterDistance - (changeInRobotOrientation * UpliftRobot.horizontalEncoderInchesPerDegreeOffset);

        robot.worldX += ((((deltaLeftDistance + deltaRightDistance) / 2.0)) * Math.sin(Math.toRadians(robot.worldAngle + (0.5 * changeInRobotOrientation)))) + (deltaHorizontal * Math.cos(Math.toRadians(robot.worldAngle + (0.5 * changeInRobotOrientation))));

        robot.worldY += ((((deltaLeftDistance + deltaRightDistance) / 2.0)) * Math.cos(Math.toRadians(robot.worldAngle + (0.5 * changeInRobotOrientation)))) - (deltaHorizontal * Math.sin(Math.toRadians(robot.worldAngle + (0.5 * changeInRobotOrientation))));

        robot.rawAngle = robot.rawAngle + changeInRobotOrientation;
        robot.worldAngle = MathFunctions.angleRestrictions(robot.rawAngle);

        initialLeftDistance = finalLeftDistance;
        initialRightDistance = finalRightDistance;
        initialCenterDistance = finalCenterDistance;

    }

    public int getLeftTicks() {
        return leftEncoder.getCurrentPosition();
    }

    public int getRightTicks() {
        return rightEncoder.getCurrentPosition();
    }

    public int getCenterTicks() {
        return -centerEncoder.getCurrentPosition();
    }

    public void setOdometryPosition(double x, double y, double angle) {
        robot.worldX = x;
        robot.worldY = y;
        robot.worldAngle = angle; // in degrees
        robot.rawAngle = angle; // in degrees
    }
}
