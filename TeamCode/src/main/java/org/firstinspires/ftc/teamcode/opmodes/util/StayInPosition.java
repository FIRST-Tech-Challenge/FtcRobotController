package org.firstinspires.ftc.teamcode.opmodes.util;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

public class StayInPosition {
    public static void stayInPose(@NonNull SampleMecanumDrive drive, Pose2d pose) {
        Pose2d currentPose = drive.getPoseEstimate();
        if (!drive.isBusy() && !currentPose.epsilonEquals(pose)) {
            drive.followTrajectoryAsync(drive.trajectoryBuilder(currentPose)
                    .lineToLinearHeading(pose)
                    .build());
        }
        drive.update();
    }
}
