package org.firstinspires.ftc.teamcode.opmodes.util;

import static org.firstinspires.ftc.teamcode.roadrunner.drive.RoadRunnerHelper.inchesToCoordinate;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

public class WallSmash {
    private static final double nextToWall = 70 - inchesToCoordinate(5.8);

    public static ElapsedTime wallSmashTimer = new ElapsedTime();

    public static void smashIntoWallSideways(@NonNull SampleMecanumDrive drive, int multiplier, double milliseconds, double direction) {
        drive.setWeightedDrivePower(new Pose2d(0, multiplier, 0));
        wallSmashTimer.reset();
        while (wallSmashTimer.milliseconds() < milliseconds) {
            drive.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        Pose2d currentPose = drive.getPoseEstimate();
        if (multiplier > 0) {
            drive.setPoseEstimate(new Pose2d(currentPose.getX(), nextToWall, direction));
        } else if (multiplier < 0) {
            drive.setPoseEstimate(new Pose2d(currentPose.getX(), -nextToWall, direction));
        }
    }

    public static void smashIntoWallSideways(@NonNull SampleMecanumDrive drive, int multiplier, double milliseconds) {
        smashIntoWallSideways(drive, multiplier, milliseconds, 0);
    }
}
