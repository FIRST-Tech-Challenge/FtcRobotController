package org.firstinspires.ftc.teamcode.tools;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class AutoDataStorage {

    // Pose transfer
    public static Pose2d currentPose = new Pose2d();

    // If we only want to drive and not test auto
    public static Boolean comingFromAutonomous = false;

    public static Boolean redSide = false;
}