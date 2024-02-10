package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {

    public static int Tile = 24;
    public static int TileInverted = -24;

    protected static double RobotX = 12.5984252; /*-inches-*/
    protected static double RobotY = 16.9291339; /*-inches-*/

    protected final static Pose2d homePose = new Pose2d(Tile/2, 3 * Tile - (RobotY/2), Math.toRadians(0));

    protected static Pose2d leftPixelSpike;
    protected static Pose2d centerPixelSpike;
    protected static Pose2d rightPixelSpike;
    protected static Pose2d randomizedBackdrop;
    protected static Vector2d stationClose;
    protected static Vector2d stationFar;
    protected static Pose2d backdrop_Unload;
    protected static Pose2d stackStation;
    protected static Pose2d parkingPose;

    protected static Integer[] leftSpikeStartingTanget = {45, 135}; //For short 45 and long 135 difference
    protected static Integer[] leftSpikeFinalTanget = {180, 0}; //For short 180 and long 0 difference
    protected static Integer[] stackStationTanget = {180, 225, 135}; // 180 for Inner 225 for Mid and Outer FROM INNER// 135 for FROM OUTER
    protected static Integer[] parkingTanget = {135, 225}; // 135 for Inner 225 for Mid and Outer

    protected static Integer leftSpikeStartingTangetValue;
    protected static Integer leftSpikeFinalTangetValue;
    protected static Integer stackStationTangetValue;
    protected static Integer parkingTangetValue;

    protected static Pose2d leftPixel_SHORT = new Pose2d((RobotY/2), Tile + (RobotX/2), Math.toRadians(180));
    protected static Pose2d centerPixel_SHORT = new Pose2d(Tile/2, Tile + (RobotY/2), Math.toRadians(90));
    protected static Pose2d rightPixel_SHORT = new Pose2d(Tile - (RobotY/2), 1.5 * Tile, Math.toRadians(0));

    protected static Pose2d leftPixel_LONG = new Pose2d(2 * TileInverted + (RobotY/2), Tile + (RobotX/2), Math.toRadians(180));
    protected static Pose2d centerPixel_LONG = new Pose2d(1.5 * TileInverted, Tile + (RobotY/2), Math.toRadians(90));
    protected static Pose2d rightPixel_LONG = new Pose2d(TileInverted - (RobotY/2), Tile, Math.toRadians(0));

    protected static Pose2d backdropLeft = new Pose2d(2.5 * Tile - (RobotY/2), 1.75 * Tile, Math.toRadians(180)); // Default
    protected static Pose2d backdropCenter = new Pose2d(2.5 * Tile - (RobotY/2), 1.5 * Tile, Math.toRadians(180));
    protected static Pose2d backdropRight = new Pose2d(2.5 * Tile - (RobotY/2), 1.25 * Tile, Math.toRadians(180)); // Default

    protected static Pose2d stationInner = new Pose2d(3 * TileInverted + (RobotY/2),Tile/2, Math.toRadians(180)); // Default
    protected static Pose2d stationMiddle = new Pose2d(3 * TileInverted + (RobotY/2),Tile, Math.toRadians(180));
    protected static Pose2d stationOuter = new Pose2d(3 * TileInverted + (RobotY/2), 1.5 * Tile, Math.toRadians(180)); // Default

    protected static Pose2d parkingInner = new Pose2d(2.5 * Tile, Tile/2, Math.toRadians(180));
    protected static Pose2d parkingMiddle = new Pose2d(2 * Tile, 1.5 * Tile, Math.toRadians(180));
    protected static Pose2d parkingOuter = new Pose2d(2.5 * Tile, 2.5 * Tile, Math.toRadians(180));

    protected static Vector2d stationClose_Inner = new Vector2d(Tile, Tile/2);
    protected static Vector2d stationFar_Inner = new Vector2d(2 * TileInverted,Tile/2);

    protected static Vector2d stationClose_Outer = new Vector2d(Tile, 2.5 * Tile);
    protected static Vector2d stationFar_Outer = new Vector2d(2 * TileInverted,2.5 * Tile);

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        randomizedBackdrop = backdropRight;
        stationClose = stationClose_Outer;
        stationFar = stationFar_Outer;
        backdrop_Unload = backdropRight;
        stackStation = stationOuter;
        leftSpikeStartingTangetValue = 0;
        leftSpikeFinalTangetValue = 0;
        stackStationTangetValue = 2;
        leftPixelSpike = leftPixel_SHORT;
        centerPixelSpike = centerPixel_SHORT;
        rightPixelSpike = rightPixel_SHORT;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(90, 90, Math.toRadians(360), Math.toRadians(360), 9.64566929)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(homePose)

                                .strafeTo(rightPixelSpike.vec())


                                .addDisplacementMarker(() -> {

                                })
                                .setTangent(Math.toRadians(270))
                                .splineToLinearHeading(randomizedBackdrop, Math.toRadians(60))
                                .addDisplacementMarker(() -> {

                                })
                                /*-------------------------------------------------------------------*/
                                /*----2+2----*/
                                /*-------------------------------------------------------------------*/
                                .setTangent(Math.toRadians(180))
                                .splineToConstantHeading(stationClose, Math.toRadians(180))
                                .lineTo(stationFar)
                                .splineToConstantHeading(stackStation.vec(), Math.toRadians(stackStationTanget[stackStationTangetValue])) //tan pair 180/225
                                .addDisplacementMarker(() -> {

                                })
                                .setReversed(true)
                                .setTangent(Math.toRadians(0))
                                .splineToConstantHeading(stationFar, Math.toRadians(0))
                                .lineTo(stationClose)
                                .addDisplacementMarker(() -> {

                                })
                                .splineToConstantHeading(backdrop_Unload.vec(), Math.toRadians(0))
                                .addDisplacementMarker(() -> {

                                })
                                /*-------------------------------------------------------------------*/
                                /*----2+4----*/
                                /*-------------------------------------------------------------------*/
                                .setTangent(Math.toRadians(180))
                                .splineToConstantHeading(stationClose, Math.toRadians(180))
                                .lineTo(stationFar)
                                .splineToConstantHeading(stackStation.vec(), Math.toRadians(stackStationTanget[stackStationTangetValue])) //tan pair 180/225
                                .addDisplacementMarker(() -> {

                                })
                                .setReversed(true)
                                .setTangent(Math.toRadians(0))
                                .splineToConstantHeading(stationFar, Math.toRadians(0))
                                .lineTo(stationClose)
                                .addDisplacementMarker(() -> {

                                })
                                .splineToConstantHeading(backdrop_Unload.vec(), Math.toRadians(0))
                                .addDisplacementMarker(() -> {

                                })
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}