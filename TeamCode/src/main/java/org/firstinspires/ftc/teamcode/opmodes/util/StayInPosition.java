package org.firstinspires.ftc.teamcode.opmodes.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

public class StayInPosition {
    public static void stayInPose(SampleMecanumDrive drive, Pose2d pose) {
        Pose2d currentPose = drive.getPoseEstimate();
        if (!drive.isBusy() && !currentPose.epsilonEquals(pose)) {
            drive.followTrajectoryAsync(drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(pose)
                    .build());
        }
    }
}
