package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

// PSEUDOCODE
public class Auton {

    private final int parkingZone;

    public Auton(int pz) {
        this.parkingZone = pz;
    }

    public void runAutonRight() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = drive.TrajectorySequenceBuilder(startPose)
                .forward(42)
                .lineToLinearHeading(new Pose2d(0, 48, Math.toRadians(45)))
                .addDisplacementMarker(() -> {
                    //first cone placement
                })
                .lineToLinearHeading(new Pose2d(0, 36, Math.toRadians(0)))
                .forward(6)
                .build();

        drive.followTrajectorySequence(trajSeq);

        if (parkingZone == 1) {
            Trajectory park = drive.trajectoryBuilder(startPose)
                    .strafeLeft(24)
                    .build();
            drive.followTrajectory(park);
        } else if (parkingZone == 3) {
            Trajectory park = drive.trajectoryBuilder(startPose)
                    .strafeRight(24)
                    .build();
            drive.followTrajectory(park);
        }
    }

    public void runAutonLeft() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = drive.TrajectorySequenceBuilder(startPose)
                .forward(42)
                .lineToLinearHeading(new Pose2d(0, 48, Math.toRadians(-45)))
                .addDisplacementMarker(() -> {
                    //first cone placement
                })
                .lineToLinearHeading(new Pose2d(0, 36, Math.toRadians(0)))
                .forward(6)
                .build();

        drive.followTrajectorySequence(trajSeq);

        if (parkingZone == 1) {
            Trajectory park = drive.trajectoryBuilder(startPose)
                    .strafeLeft(24)
                    .build();
            drive.followTrajectory(park);
        } else if (parkingZone == 3) {
            Trajectory park = drive.trajectoryBuilder(startPose)
                    .strafeRight(24)
                    .build();
            drive.followTrajectory(park);
        }
    }

    public void runAutoasdfasdfnLeft() {
        new TrajectoryBuilder(new Pose2d())
            .forward(42)
            .lineToLinearHeading(new Pose2d(0, 48, Math.toRadians(-45)))
            .lineToLinearHeading(new Pose2d(0, 36, Math.toRadians(0)))
            .forward(6)
            .build();

        placeFirstCone();

        if (parkingZone == 1) {
            new TrajectoryBuilder(new Pose2d())
                .strafeRight(24)
                .build();

        } else if (parkingZone == 3) {
            new TrajectoryBuilder(new Pose2d())
                .strafeLeft(24)
                .build();
        }
    }

    public void placeFirstCone() {
        // places first cone
    }

    public void pickUpSecondCone() {
        // picks up second cone
    }

    public void placeSecondCone() {
        // places second cone
    }
}
