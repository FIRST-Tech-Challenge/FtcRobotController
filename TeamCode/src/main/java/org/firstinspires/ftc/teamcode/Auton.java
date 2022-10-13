package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

// PSEUDOCODE
public class Auton {

    private final int parkingZone;

    public Auton(int pz) {
        this.parkingZone = pz;
    }

    public void runAutonRight(SampleMecanumDrive getDrive) {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(18)
                .lineToLinearHeading(new Pose2d(24, 0, Math.toRadians(-45)))
                .addDisplacementMarker(() -> {
                    //first cone placement
                })
                .lineToLinearHeading(new Pose2d(18, 0, Math.toRadians(0)))
                .back(6)
                .build();

        drive.followTrajectorySequence(trajSeq);

        if (parkingZone == 1) {
            Trajectory park = drive.trajectoryBuilder(new Pose2d())
                    .strafeLeft(24)
                    .build();
            drive.followTrajectory(park);
        } else if (parkingZone == 3) {
            Trajectory park = drive.trajectoryBuilder(new Pose2d())
                    .strafeRight(24)
                    .build();
            drive.followTrajectory(park);
        }
    }

    public void runAutonLeft(SampleMecanumDrive getDrive) {

        SampleMecanumDrive drive = getDrive;

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(18)
                .lineToLinearHeading(new Pose2d(24, 0, Math.toRadians(45)))
                .addDisplacementMarker(() -> {
                    //first cone placement
                })
                .lineToLinearHeading(new Pose2d(18, 0, Math.toRadians(0)))
                .back(6)
                .build();

        drive.followTrajectorySequence(trajSeq);

        if (parkingZone == 1) {
            Trajectory park = drive.trajectoryBuilder(new Pose2d())
                    .strafeLeft(24)
                    .build();
            drive.followTrajectory(park);
        } else if (parkingZone == 3) {
            Trajectory park = drive.trajectoryBuilder(new Pose2d())
                    .strafeRight(24)
                    .build();
            drive.followTrajectory(park);
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
