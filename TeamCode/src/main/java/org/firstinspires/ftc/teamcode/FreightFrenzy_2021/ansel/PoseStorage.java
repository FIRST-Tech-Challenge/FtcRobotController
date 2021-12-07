package org.firstinspires.ftc.teamcode.FreightFrenzy_2021.ansel;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class PoseStorage {
    public static Pose2d currentPose = new Pose2d();
    public static driveMethod.poseState state = driveMethod.poseState.UNKNOWN;
}
