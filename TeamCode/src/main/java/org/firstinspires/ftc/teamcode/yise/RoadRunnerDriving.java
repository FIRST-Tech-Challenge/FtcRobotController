package org.firstinspires.ftc.teamcode.yise;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class RoadRunnerDriving {

    SampleMecanumDrive drive;
    // Note: we make these public so the calling code can access and use these variables

    // Used to track slow-mode versus normal mode
    public Speeds currentSpeed;
    double speedMultiplier;

    public enum Speeds {
        SLOW,
        NORMAL
    }

    //Declare the constructor for the class
    public RoadRunnerDriving(HardwareMap hardwareMap) {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
        //drive.setPoseEstimate(PoseStorage.currentPose);

        // set default value for speed
        currentSpeed = Speeds.NORMAL;
        speedMultiplier = 1;
    }


    public void updateMotorsFromStick(Gamepad gamepad) {
        drive.setWeightedDrivePower(new Pose2d(
                -gamepad.left_stick_y * speedMultiplier,
                -gamepad.left_stick_x * speedMultiplier,
                -gamepad.right_stick_x * speedMultiplier
        ));
    }

    //Toggles speed
    public void toggleSlowMode(Speeds targetSpeed) {

        // Set the speedMultiplier in case of SLOW mode
        if (currentSpeed == Speeds.SLOW) {
            currentSpeed = Speeds.NORMAL;
            speedMultiplier = 1;
        } else {
            currentSpeed = Speeds.SLOW;
            speedMultiplier = 0.5;
        }
    }

    public void update() {
        drive.update();
    }

    public Pose2d getPosition() {
        return drive.getPoseEstimate();
    }

    public void calibratePos(AprilTagDetection detection) {
        if (Parameters.allianceColor == Parameters.Color.RED) {
            drive.setPoseEstimate(new Pose2d(55.5 - detection.ftcPose.y, -(36.25 + detection.ftcPose.x), drive.getRawExternalHeading()));
        } else {
            double driveHeading = drive.getPoseEstimate().getHeading();
            drive.setPoseEstimate(new Pose2d(55.5 - detection.ftcPose.y, 36.25 + detection.ftcPose.x, driveHeading));
        }
    }

    public void navigateToCorner() {
        if (!drive.isBusy()) {
            if (Parameters.allianceColor == Parameters.Color.RED) {

                Trajectory navigateToCorner = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-51, 51, Math.toRadians(135)))
                        .build();
                drive.followTrajectory(navigateToCorner);

            } else {

                Trajectory navigateToCorner = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-51, -51, Math.toRadians(-135)))
                        .build();
                drive.followTrajectory(navigateToCorner);

            }
        }
    }

    public void dropPixelNear() {

        if (!drive.isBusy()) {
            if (Parameters.allianceColor == Parameters.Color.RED) {

                Trajectory dropPixelNear = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(50.5,-36, Math.toRadians(180)))
                        .build();
                drive.followTrajectory(dropPixelNear);

            } else {

                Trajectory dropPixelNear = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(50.5,36, Math.toRadians(180)))
                        .build();
                drive.followTrajectory(dropPixelNear);

            }
        }
    }

    public void dropPixelMid() {

        if (!drive.isBusy()) {
            if (Parameters.allianceColor == Parameters.Color.RED) {

                Trajectory dropPixelMid = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(46,-36, Math.toRadians(180)))
                        .build();
                drive.followTrajectory(dropPixelMid);

            } else {

                Trajectory dropPixelMid = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(46,36, Math.toRadians(180)))
                        .build();
                drive.followTrajectory(dropPixelMid);

            }
        }
    }

    public void dropPixelFar() {

        if (!drive.isBusy()) {
            if (Parameters.allianceColor == Parameters.Color.RED) {

                Trajectory dropPixelFar = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(39,-36, Math.toRadians(180)))
                        .build();
                drive.followTrajectory(dropPixelFar);

            } else {

                Trajectory dropPixelFar = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(39,36, Math.toRadians(180)))
                        .build();
                drive.followTrajectory(dropPixelFar);

            }
        }
    }
}

