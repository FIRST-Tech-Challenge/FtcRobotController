package org.firstinspires.ftc.teamcode.yise;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

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
                gamepad.left_stick_y * speedMultiplier,
                gamepad.left_stick_x * speedMultiplier,
                gamepad.right_stick_x * speedMultiplier
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

    public void navigateToCornerRed() {
        if (drive.getPoseEstimate().getX() < -24) {
            if (!drive.isBusy()) {
                Trajectory navigateToCornerRed = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-53, 53, Math.toRadians(135)))
                        .build();
                drive.followTrajectory(navigateToCornerRed);
            }
        }
    }

    public void pixelDropRedNear() {

        if (!drive.isBusy()) {
            Trajectory pixelDropRedNear = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(40,-36, Math.toRadians(180)))
                    .build();
            drive.followTrajectory(pixelDropRedNear);
        }
    }

    public void pixelDropBlueFar() {

        if (!drive.isBusy()) {
            Trajectory pixelDropBlueFar = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(31,36, Math.toRadians(180)))
                    .build();
            drive.followTrajectory(pixelDropBlueFar);
        }
    }

    public void pixelDropBlueNear() {

        if (!drive.isBusy()) {
            Trajectory pixelDropRed = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(40,36, Math.toRadians(180)))
                    .build();
            drive.followTrajectory(pixelDropRed);
        }
    }
}

