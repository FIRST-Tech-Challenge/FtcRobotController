package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;

// PSEUDOCODE
public class Auton {

    private int parkingZone;

    public Auton(int parkingZone) {

        this.parkingZone = parkingZone;
    }

    public void runAutonRight1() {

        new TrajectoryBuilder(new Pose2d())
            .forward(18)
            .lineToLinearHeading(new Pose2d(0, 48, Math.toRadians(-45)))
            .lineToLinearHeading(new Pose2d(0, 36, Math.toRadians(0)))
            .forward(6)
            .build()

        placeFirstCone();

        if (parkingZone == 1) {
            new TrajectoryBuilder(new Pose2d())
                .strafeLeft(24)
                .build
        } else if (parkingZone == 3) {
            new TrajectoryBuilder(new Pose2d())
                .strafeRight(24)
                .build
        }
    }

    public void runAutonLeft1() {
        new TrajectoryBuilder(new Pose2d())
            .forward(18)
            .lineToLinearHeading(new Pose2d(0, 48, Math.toRadians(-45)))
            .lineToLinearHeading(new Pose2d(0, 36, Math.toRadians(0)))
            .forward(6)
            .build()

        placeFirstCone();

        if (parkingZone == 1) {
            new TrajectoryBuilder(new Pose2d())
                .strafeRight(24)
                .build()

        } else if (parkingZone == 3) {
            new TrajectoryBuilder(new Pose2d())
                .strafeLeft(24)
                .build()
        }
    }

    public void runAutonRight2() {
        new TrajectoryBuilder(new Pose2d())
            .forward(18)
            .lineToLinearHeading(new Pose2d(0, 48, Math.toRadians(45)))
            .build()

        placeFirstCone();

        new TrajectoryBuilder(new Pose2d())
            .backward(6)
            .lineToLinearHeading(new Pose2d(24, 48, Math.toRadians(90)))
            .build

        pickUpSecondCone();

        new TrajectoryBuilder(new Pose2d())
            .lineToLinearHeading(new Pose2d(24, 42, Math.toRadians(0)))
            .lineToLinearHeading(new Pose2d(24, 24, Math.toRadians(45)))
            .build

        placeSecondCone();

        new TrajectoryBuilder(new Pose2d(24, 30, Math.toRadians(-90)))
            .lineToLinearHeading(new)
            .strafeLeft(18)
            .build

        if(parkingZone == 1) {
            new TrajectoryBuilder(new Pose2d())
                .strafeLeft(24)
                .build
        }
    }

    public void runAutonRight2() {
        new TrajectoryBuilder(new Pose2d())
                .forward(18)
                .lineToLinearHeading(new Pose2d(0, 48, Math.toRadians(-45)))
                .build()

        placeFirstCone();

        new TrajectoryBuilder(new Pose2d())
                .backward(6)
                .lineToLinearHeading(new Pose2d(-24, 48, Math.toRadians(-90)))
                .build

        pickUpSecondCone();

        new TrajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(-24, 42, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-24, 24, Math.toRadians(45)))
                .build

        placeSecondCone();

        new TrajectoryBuilder(new Pose2d(-24, 30, Math.toRadians(90)))
                .lineToLinearHeading(new)
                .strafeLeft(18)
                .build

        if(parkingZone == 3) {
            new TrajectoryBuilder(new Pose2d())
                    .strafeRight(24)
                    .build
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
