package org.firstinspires.ftc.teamcode.SubSystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

public class HzGameField {
    // Declare a target vector you'd like your bot to align with
    // Can be any x/y coordinate of your choosing
    public static final Vector2d ORIGIN = new Vector2d(0,0);
    public static final Vector2d BLUE_TOWER_GOAL = new Vector2d(72,42);
    public static final Vector2d BLUE_POWERSHOT1 = new Vector2d(72,17.75);
    public static final Vector2d BLUE_POWERSHOT2 = new Vector2d(72,10.25);
    public static final Vector2d BLUE_POWERSHOT3 = new Vector2d(72,2.75);
    public static final Vector2d RED_POWERSHOT3 = new Vector2d(72,-2.75);
    public static final Vector2d RED_POWERSHOT2 = new Vector2d(72,-10.25);
    public static final Vector2d RED_POWERSHOT1 = new Vector2d(72,-17.75);
    public static final Vector2d RED_TOWER_GOAL = new Vector2d(72,-36);

    public static final Pose2d BLUE_INNER_START_LINE =  new Pose2d(-57,26,Math.toRadians(90));
    public static final Pose2d BLUE_OUTER_START_LINE =  new Pose2d(-57,44,Math.toRadians(-90));
    public static final Pose2d RED_INNER_START_LINE =  new Pose2d(-57,-26,Math.toRadians(-90));
    public static final Pose2d RED_OUTER_START_LINE =  new Pose2d(-57,-44,Math.toRadians(90));

    public static final Pose2d calibPoint = new Pose2d(0,0,Math.toRadians(0));

    public enum PLAYING_ALLIANCE{
        RED_ALLIANCE,
        BLUE_ALLIANCE,
    }
    public static PLAYING_ALLIANCE playingAlliance = PLAYING_ALLIANCE.BLUE_ALLIANCE;

    public enum START_POSITION{
        INNER,
        OUTER
    }
    public static START_POSITION startPosition = START_POSITION.INNER;

    public static double ALLIANCE_FACTOR = 1;

    public enum TARGET_ZONE{
        A,
        B,
        C,
        UNKNOWN;
    };

    public static final Vector2d BLUE_TARGET_ZONE_A = new Vector2d(12,60);
    public static final Vector2d BLUE_TARGET_ZONE_B = new Vector2d(36,36);
    public static final Vector2d BLUE_TARGET_ZONE_C = new Vector2d(60,60);

    public static final Vector2d RED_TARGET_ZONE_A = new Vector2d(12,-60);
    public static final Vector2d RED_TARGET_ZONE_B = new Vector2d(36,-36);
    public static final Vector2d RED_TARGET_ZONE_C = new Vector2d(60,-60);

    //Static fields to pass Pos from Autonomous to TeleOp
    public static boolean poseSetInAutonomous = false;
    public static Pose2d currentPose = new Pose2d();

}
