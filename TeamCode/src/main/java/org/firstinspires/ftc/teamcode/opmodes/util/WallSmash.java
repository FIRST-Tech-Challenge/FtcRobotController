package org.firstinspires.ftc.teamcode.opmodes.util;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

public class WallSmash {
    public static ElapsedTime wallSmashTimer = new ElapsedTime();

    public static void smashIntoWall(@NonNull SampleMecanumDrive drive, int multiplier, double milliseconds) {
        drive.setWeightedDrivePower(new Pose2d(0, multiplier, 0));
        wallSmashTimer.reset();
        while (wallSmashTimer.milliseconds() < milliseconds) {
            drive.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
    }
}
