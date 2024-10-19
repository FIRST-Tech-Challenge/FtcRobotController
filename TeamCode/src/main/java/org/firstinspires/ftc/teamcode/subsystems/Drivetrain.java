package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Drivetrain extends SubsystemBase {
//    private static int kFrontLeft = 0;
//    private static int kFrontRight = 0;
//    private static int kBackLeft = 0;
//    private static int kBackRight = 0;

    private Motor frontLeft;
    private Motor backLeft;
    private Motor frontRight;
    private Motor backRight;

    private RevIMU imu;

    private MecanumDrive mecanum;

    private Telemetry telemetry;


//    private static Drivetrain instance;
//    private MecanumDrive mecanumDrive;
    public Drivetrain(HardwareMap hmap, Telemetry telemetry) {
        // get motors for drivetrain
        this.frontLeft = new Motor(hmap, "frontLeft");
        this.backLeft = new Motor(hmap, "backLeft");
        this.frontRight = new Motor(hmap, "frontRight");
        this.backRight = new Motor(hmap, "backRight");

        this.mecanum = new MecanumDrive(false, frontLeft, frontRight,
                backLeft, backRight);

        this.frontLeft.setInverted(true);
        this.backLeft.setInverted(true);

        this.imu = new RevIMU(hmap);
        this.imu.init();
//        mecanumDrive = new MecanumDrive(hmap,pose);
        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
        telemetry.addData("Heading", this.imu.getRotation2d().getDegrees());
    }

    public void driveFieldCentric(double strafeSpeed, double forwardSpeed, double turn) {
        double deg = this.imu.getRotation2d().getDegrees();
        mecanum.driveRobotCentric(-strafeSpeed, forwardSpeed, turn);
    }

//    private double clipRange(double value) {
//        return value <= -1.0 ? -1.0
//                : value >= 1.0 ? 1.0
//                : value;
//    }

//    /**
//     * Normalize the wheel speeds
//     */
//    private void normalize(double[] wheelSpeeds, double magnitude) {
//        double maxMagnitude = Math.abs(wheelSpeeds[0]);
//        for (int i = 1; i < wheelSpeeds.length; i++) {
//            double temp = Math.abs(wheelSpeeds[i]);
//            if (maxMagnitude < temp) {
//                maxMagnitude = temp;
//            }
//        }
//        for (int i = 0; i < wheelSpeeds.length; i++) {
//            wheelSpeeds[i] = (wheelSpeeds[i] / maxMagnitude) * magnitude;
//        }
//
//    }
//
//    /**
//     * Normalize the wheel speeds
//     */
//    private void normalize(double[] wheelSpeeds) {
//        double maxMagnitude = Math.abs(wheelSpeeds[0]);
//        for (int i = 1; i < wheelSpeeds.length; i++) {
//            double temp = Math.abs(wheelSpeeds[i]);
//            if (maxMagnitude < temp) {
//                maxMagnitude = temp;
//            }
//        }
//        if (maxMagnitude > 1) {
//            for (int i = 0; i < wheelSpeeds.length; i++) {
//                wheelSpeeds[i] = (wheelSpeeds[i] / maxMagnitude);
//            }
//        }
//    }
//
//    @Override
//    public void periodic() {
//        this.mecanumDrive.updatePoseEstimate();
//    }
//
//    public void driveArcade(double forwardSpeed, double strafeSpeed, double turnSpeed) {
//        this.fieldCentricDrive(forwardSpeed, strafeSpeed, turnSpeed, 0.0);
//    }
//
//    public void driveFieldCentric(double forwardSpeed, double strafeSpeed, double turnSpeed) {
//        Rotation2d gyroAngle = this.mecanumDrive.pose.heading;
//        this.fieldCentricDrive(forwardSpeed, strafeSpeed, turnSpeed, gyroAngle.toDouble());
//    }
//
//    private void fieldCentricDrive(double forwardSpeed, double strafeSpeed, double turnSpeed, double gyroAngle) {
//
//        strafeSpeed = clipRange(strafeSpeed);
//        forwardSpeed = clipRange(forwardSpeed);
//        turnSpeed = clipRange(turnSpeed);
//        Vector2d input = new Vector2d(strafeSpeed, forwardSpeed);
//        input = input.rotateBy(-gyroAngle);
//
//        double theta = input.angle();
//
//        double[] wheelSpeeds = new double[4];
//        wheelSpeeds[kFrontLeft] = Math.sin(theta + Math.PI / 4);
//        wheelSpeeds[kFrontRight] = Math.sin(theta - Math.PI / 4);
//        wheelSpeeds[kBackLeft] = Math.sin(theta - Math.PI / 4);
//        wheelSpeeds[kBackRight] = Math.sin(theta + Math.PI / 4);
//
//        normalize(wheelSpeeds, input.magnitude());
//
//        wheelSpeeds[kFrontLeft] += turnSpeed;
//        wheelSpeeds[kFrontRight] -= turnSpeed;
//        wheelSpeeds[kBackLeft] += turnSpeed;
//        wheelSpeeds[kBackRight] -= turnSpeed;
//
//        normalize(wheelSpeeds);
//
//        this.mecanumDrive.leftFront.setPower(wheelSpeeds[kFrontLeft]);
//        this.mecanumDrive.rightFront.setPower(wheelSpeeds[kFrontRight]);
//        this.mecanumDrive.leftBack.setPower(wheelSpeeds[kBackLeft]);
//        this.mecanumDrive.rightBack.setPower(wheelSpeeds[kBackRight]);
//    }
//
//    public TrajectoryActionBuilder getTrajectoryBuilder(Pose2d initalPose) {
//        return this.mecanumDrive.actionBuilder(initalPose);
//    }
}
