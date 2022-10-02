package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;

// PSEUDOCODE
public class Auton {

    private int parkingZone;

    public Auton(int parkingZone) {
        this.parkingZone = parkingZone;
    }

    public void runAuton() {

        new TrajectoryBuilder(new Pose2d())
            .forward(18)
            .lineToLinearHeading(new Pose2d(0, 48, Math.toRadians(45)))
            .build()

        placeFirstCone();

        new TrajectoryBuilder(new Pose2d())
            //.lineToLinearHeading(new Pose2d(0, 36, Math.toRadians(0)))
            .build()

        if(parkingZone == 1) {
            new TrajectoryBuilder(newPose2d())
                .strafeLeft(24)
                .build
        } else if(parkingZone == 3) {
            new TrajectoryBuilder(newPose2d())
                    .strafeRight(24)
                    .build
        }
    }

    public void placeFirstCone() {
        // places cone
    }
}
