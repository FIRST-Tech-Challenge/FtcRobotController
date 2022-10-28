package org.firstinspires.ftc.teamcode.AutoOpMode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

/**
 * Static Class to define Gamefield Vector positions.
 * These are used in start Position estimates and in automatic targetic.
 *
 * The static class also has PosStorage defined, to pass the last position in autonomous mode
 * to following TeleOp mode
 */
public class GameField {
    // Declare a target vector you'd like your bot to align with
    // Can be any x/y coordinate of your choosing
    public static final Vector2d ORIGIN = new Vector2d(0,0);
    public static final Pose2d ORIGINPOSE = new Pose2d(0,0,Math.toRadians(0));

    // Declare and assign starting pose of robot
    //TODO: Update start position correctly.
    public static final Pose2d BLUE_ALLIANCE_BLUE_TERMINAL =  new Pose2d(-60,-36,Math.toRadians(180));
    public static final Pose2d BLUE_ALLIANCE_RED_TERMINAL =  new Pose2d(-60,36,Math.toRadians(180));
    public static final Pose2d RED_ALLIANCE_RED_TERMINAL =  new Pose2d(60,-36,Math.toRadians(0));
    public static final Pose2d RED_ALLIANCE_BLUE_TERMINAL =  new Pose2d(60,36,Math.toRadians(0));

    //Define and declare Playing Alliance
    public enum PLAYING_ALLIANCE{
        RED_ALLIANCE,
        BLUE_ALLIANCE,
    }
    public static PLAYING_ALLIANCE playingAlliance = PLAYING_ALLIANCE.BLUE_ALLIANCE;
    public static double ALLIANCE_FACTOR = 1;


    //Define and declare Robot Starting Locations
    public enum START_POSITION{
        RED_TERMINAL,
        BLUE_TERMINAL,
        ORIGIN
    }

    //Change the start position to whatever you desire: below is default
    public static START_POSITION startPosition = START_POSITION.RED_TERMINAL;

    public enum PARKING_LOCATION{

    }

    public enum AUTONOMOUS_ROUTE {
        THROUGH_BARRIER,
        ALONG_WALL,
        NOT_APPLICABLE
    }

    public static boolean END_PARKING_FACING_SHARED_SHIPPING_HUB = false;

    //Define targets for Vision to determine Autonomous mode action based on camera detection
    public enum VISION_IDENTIFIED_TARGET {
        LEVEL1,
        LEVEL2,
        LEVEL3,
        UNKNOWN;
    };

    public enum VISION_IDENTIFIER{
        DUCK,
        MARKER
    };
    public static VISION_IDENTIFIER visionIdentifier = VISION_IDENTIFIER.DUCK;

    //Static fields to pass Pos from Autonomous to TeleOp
    public static boolean poseSetInAutonomous = false;
    public static Pose2d currentPose = new Pose2d();

}
