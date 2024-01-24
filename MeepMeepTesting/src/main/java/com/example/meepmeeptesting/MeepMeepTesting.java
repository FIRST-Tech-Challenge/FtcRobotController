package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {

    protected static int Tile = 24; //inch
    protected static int TileInverted = -24; //inch
    protected static int RobotX = 13; //inch
    protected static int RobotY = 14; //inch
    protected static double BackdropDistance = 10.5; //inch
    protected static double BackdropDistance_FIRST = 11; //inch
    protected static int StationDistance = 9; //inch
    protected static int StationDistance_SMALL = 5; //inch
    protected static int Invert = 1; //false

    public static Vector2d leftPixel_LOW = new Vector2d(1.25 * TileInverted * Invert - (RobotX/2 * Invert), 0 - (RobotY/2));
    public static Vector2d rightPixel_LOW = new Vector2d(1.25 * TileInverted * Invert - (RobotX/2 * Invert), TileInverted + (RobotY/2));

    public static Vector2d leftPixel_HIGH = new Vector2d(1.25 * TileInverted - (RobotX/2), 2 * Tile - (RobotY/2));
    public static Vector2d rightPixel_HIGH = new Vector2d(1.25 * TileInverted - (RobotX/2), Tile + (RobotY/2));

    public static Pose2d backdrop = new Pose2d(1.5 * TileInverted * Invert, 2 * TileInverted + (RobotY/2) - BackdropDistance_FIRST, Math.PI/2);
    public static Pose2d backdrop_INNER = new Pose2d(1.25 * TileInverted * Invert, 2 * TileInverted + (RobotY/2) - BackdropDistance, Math.PI/2);
    public static Pose2d backdrop_MID = new Pose2d(1.5 * TileInverted * Invert, 2 * TileInverted + (RobotY/2) - BackdropDistance, Math.PI/2);
    public static Pose2d backdrop_OUTER = new Pose2d(1.75 * TileInverted * Invert, 2 * TileInverted + (RobotY/2) - BackdropDistance, Math.PI/2);

    public static Pose2d station_INNER = new Pose2d(TileInverted/2 * Invert, 2 * Tile, Math.PI/2);
    public static Pose2d station_OUTER = new Pose2d(2.5 * TileInverted * Invert, 1.5 * Tile, Math.PI/2);

    public static Pose2d middle_LOW = new Pose2d(1.5 * TileInverted * Invert, TileInverted/2, Math.toRadians(180));
    public static Pose2d middle_HIGH = new Pose2d(1.5 * TileInverted * Invert, 1.5 * Tile, Math.toRadians(-90));

    public static Pose2d station_POS_OUTER = new Pose2d(1.5 * TileInverted * Invert, 2.5 * Tile - (RobotY/2) - StationDistance, Math.PI/2);
    public static Pose2d station_POS_MID = new Pose2d(TileInverted * Invert, 2.5 * Tile - (RobotY/2) - StationDistance, Math.PI/2);
    public static Pose2d station_POS_INNER = new Pose2d(TileInverted/2 * Invert + 2, 3 * Tile - (RobotY/2) - StationDistance, Math.PI/2);

    public static Pose2d station_POS_INNER_SMALL = new Pose2d(TileInverted/2 * Invert+2, 2.7 * Tile - (RobotY/2) - StationDistance_SMALL, Math.PI/2);


    protected static double starting_pos_error_X = 1;//inch
    protected static double starting_pos_error_Y = 1;//inch

    protected static Pose2d homePose_LOW_RED = new Pose2d((3 * TileInverted) + (RobotY/2) + starting_pos_error_X,0,Math.toRadians(180));
    public static Pose2d homePose_HIGH_RED = new Pose2d((3 * TileInverted) + (RobotY/2),(Tile * 1.5),Math.toRadians(180));


    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);



        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(homePose_LOW_RED)
                                .setTangent(-45)
                                .splineToConstantHeading(new Vector2d((2 * TileInverted * Invert), TileInverted/2), Math.toRadians(0))
                                .splineTo(leftPixel_LOW, Math.PI/2)
                                .lineTo(middle_LOW.vec())
                                .turn(Math.toRadians(180))
                                .build()

                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}