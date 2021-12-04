package org.firstinspires.ftc.teamcode.FreightFrenzy_2021.ansel;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.FreightFrenzy_2021.ansel.driveMethod;

public class PoseStorage {
    public static Pose2d currentPose = new Pose2d();
    public static driveMethod.poseState state = driveMethod.poseState.UNKNOWN;
}
