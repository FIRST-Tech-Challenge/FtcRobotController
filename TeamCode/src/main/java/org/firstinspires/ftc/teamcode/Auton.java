package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

// PSEUDOCODE
public class Auton {

    private static int parkingZone;//1, 2, 3
    private boolean direction;//true = left, false = right

    public Auton(boolean left) {
        this.direction = left;
    }

    public void runAuton(int pz, SampleMecanumDrive getDrive)
    {
        parkingZone = pz;

        int angle = -45;
        if(direction)
        {
            angle = 45;
        }
        SampleMecanumDrive drive = getDrive;

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(18)
                .lineToLinearHeading(new Pose2d(24, 0, Math.toRadians(angle)))
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
