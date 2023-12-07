package org.firstinspires.ftc.teamcode.roadrunner.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import java.util.Collections;
import java.util.List;

/**
 * Simple static field serving as a storage medium for the bot's pose.
 * This allows different classes/opmodes to set and read from a central source of truth.
 * A static field allows data to persist between opmodes.
 */
public class PoseStorage {
    public static Pose2d currentPose = new Pose2d(0,0,0);
    public static Pose2d currentVelocity = new Pose2d(0,0,0);
    public static Pose2d currentPOVVelocity = new Pose2d(0,0,0);
    public static List<Double> lastWheelPositions = Collections.emptyList();
    public static void setPose(Pose2d pose){
        currentPose = pose;
//        lastWheelPositions = Collections.emptyList();
    }
}