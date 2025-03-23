package com.kalipsorobotics.actions.drivetrain;

import static java.lang.Math.PI;

import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import org.opencv.core.Point;
//TODO make imu reset at start
public class FieldOrientedDrive {
    OpModeUtilities opModeUtilities;
    IMU imu;
    DriveTrain driveTrain;
    DriveAction driveAction;
    public FieldOrientedDrive(IMU imu1, DriveTrain driveTrain1, OpModeUtilities opModeUtilities1) {
        imu = imu1;
        driveTrain = driveTrain1;
        driveAction = new DriveAction(driveTrain);
        opModeUtilities = opModeUtilities1;
    }

    public IMU getImu() {
        return imu;
    }
    private double angleCalculator(double x, double y) {
        //calculating degrees in radians
        double r = Math.atan2(y, x);
        //converting radians to degrees
        return r * 180 / PI;
    }
    private double calculateRobotDir(Gamepad gamepad){
        //getting imu heading
        double heading = imu.getRobotYawPitchRollAngles().getYaw();
        double gamepadx = gamepad.left_stick_x;
        double gamepady = gamepad.left_stick_y;
        //calculating degrees
        double gamepadHeading = angleCalculator(gamepadx, gamepady);
        return gamepadHeading-heading;
        //robot intended direction
    }
    private double getPointDistanceFromOrigin(Gamepad gamepad) {
        Point p = new Point(gamepad.left_stick_x, gamepad.left_stick_y);
        double distanceSqr = (p.x*p.x)+(p.y*p.y);
        //calculates distance of point to (0,0)
        return Math.sqrt(distanceSqr);
    }
//    private void initializeIMU(){
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
//    }
    public Point reverseCalculate(double angle, double distance) {

        double angleInRadians = Math.toRadians(angle);
        //calculates point from degrees
        double x = distance * Math.sin(angleInRadians);
        double y = distance * Math.cos(angleInRadians);

        return new Point(x, y);
    }
    public void drive(Gamepad gamepad) {
        double robotIntendedPos = calculateRobotDir(gamepad);
        Point fixedGamepadValue = reverseCalculate(robotIntendedPos, getPointDistanceFromOrigin(gamepad));
        driveAction.moveWithXYValues(fixedGamepadValue.x, fixedGamepadValue.y, gamepad);
    }
}