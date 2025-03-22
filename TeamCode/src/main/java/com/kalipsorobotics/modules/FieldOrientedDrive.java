package com.kalipsorobotics.modules;

import static java.lang.Math.PI;

import com.kalipsorobotics.actions.drivetrain.DriveAction;
import com.kalipsorobotics.localization.SparkfunOdometry;
import com.kalipsorobotics.math.Position;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.opencv.core.Point;

public class FieldOrientedDrive {
    SparkfunOdometry sparkfunOdometry;
    DriveAction driveAction;
    DcMotor bLeft;
    DcMotor bRight;
    DcMotor fLeft;
    DcMotor fRight;
    public FieldOrientedDrive(SparkfunOdometry sparkfunOdometry, DriveTrain driveTrain, OpModeUtilities opModeUtilities) {
        sparkfunOdometry = sparkfunOdometry;

        driveAction = new DriveAction(driveTrain);
    }

    public SparkfunOdometry getSparkfunOdometry() {
        return sparkfunOdometry;
    }
    private double angleCalculator(double x, double y) {
        double r = Math.atan2(y, x);
        double degrees = r * 180 / PI;
        return degrees;
    }
    private double calculateRobotDir(Gamepad gamepad){
        sparkfunOdometry.updatePosition();
        Position p = sparkfunOdometry.getCurrentPosition();
        double heading = p.getTheta();
        double gamepadx = gamepad.left_stick_x;
        double gamepady = gamepad.left_stick_y;
        double gamepadHeading = angleCalculator(gamepadx, gamepady);
        double robotIntendedDirection = gamepadHeading-heading;
        return robotIntendedDirection;
    }
    private double getPointDistanceFromOrigin(Gamepad gamepad) {
        Point p = new Point(gamepad.left_stick_x, gamepad.left_stick_y);
        double distanceSqr = (p.x*p.x)+(p.y*p.y);
        double distance = Math.sqrt(distanceSqr);
        return distance;
    }
    public Point reverseCalculate(double angle, double distance) {

        double angleInRadians = Math.toRadians(angle);

        double x = distance * Math.sin(angleInRadians);
        double y = distance * Math.cos(angleInRadians);

        return new Point(x, y);
    }
    public void drive(Gamepad gamepad) {
        double robotIntendedPos = calculateRobotDir(gamepad);
        Point fixedGamepadValue = reverseCalculate(robotIntendedPos, getPointDistanceFromOrigin(gamepad));
        driveAction.move(gamepad);
    }
}