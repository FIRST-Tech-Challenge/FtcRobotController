package com.kalipsorobotics.modules;

import static java.lang.Math.PI;

import com.kalipsorobotics.actions.drivetrain.DriveAction;
import com.kalipsorobotics.localization.SparkfunOdometry;
import com.kalipsorobotics.math.Position;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.opencv.core.Point;

public class FieldOrientedDrive {
    OpModeUtilities opModeUtilities;
    IMU imu;
    DriveTrain driveTrain;
    DriveAction driveAction;
    public FieldOrientedDrive(IMU imu, DriveTrain driveTrain, OpModeUtilities opModeUtilities) {
        imu = imu;
        driveTrain = driveTrain;
        driveAction = new DriveAction(driveTrain);
        opModeUtilities = opModeUtilities;
    }

    public IMU getImu() {
        return imu;
    }
    private double angleCalculator(double x, double y) {
        double r = Math.atan2(y, x);
        double degrees = r * 180 / PI;
        return degrees;
    }
    private double calculateRobotDir(Gamepad gamepad){
        double heading = imu.getRobotYawPitchRollAngles().getYaw();
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
    private void initializeIMU(){
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample OpMode
//        parameters.loggingEnabled      = true;
//        parameters.loggingTag          = "IMU";
//        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
//
//        imu = opModeUtilities.getHardwareMap().get(BNO055IMU.class, "imu");
//        imu.initialize(parameters);
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
        driveAction.moveWithXYValues(fixedGamepadValue.x, fixedGamepadValue.y);
    }
}