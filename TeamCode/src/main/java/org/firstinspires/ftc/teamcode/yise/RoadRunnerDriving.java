package org.firstinspires.ftc.teamcode.yise;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class RoadRunnerDriving {

    SampleMecanumDrive drive;
    // Note: we make these public so the calling code can access and use these variables
    public IMU imu;

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
        /*PoseStorage.currentPose*/
        drive.setPoseEstimate(PoseStorage.currentPose);

        // set default value for speed
        currentSpeed = Speeds.NORMAL;
        speedMultiplier = 1;

        imu = hardwareMap.get(IMU.class, "imu");


        //Field orientation stuff
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.DOWN;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        imu.resetYaw();
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

    public void pixelDropRedFar() {

        if (!drive.isBusy()) {
            Trajectory pixelDropRedFar = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(31,-36, Math.toRadians(180)))
                    .build();
            drive.followTrajectory(pixelDropRedFar);
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

