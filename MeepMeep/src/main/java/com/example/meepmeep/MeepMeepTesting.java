package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {

    public static double Tile = 24; /*-inches-*/
    public static double TileInverted = -24; /*-inches-*/
    public static double RobotX = 12.6; /*-inches-*/
    public static double RobotY = 18; /*-inches-*/
    public static double BackdropDistance = 0; /*-inches-*/

    protected static Pose2d HomePose_SHORT = new Pose2d(Tile/2, 3 * Tile - (RobotY/2), Math.toRadians(270));
    protected static Pose2d HomePose_LONG = new Pose2d(1.5 * TileInverted, 3 * TileInverted + (RobotY/2), Math.toRadians(90));

    protected static Integer[] leftSpikeStartingTanget = {315, 225}; //For short 45 and long 135 difference
    protected static Integer[] leftSpikeFinalTanget = {180, 0}; //For short 180 and long 0 difference
    protected static Integer[] stackStationTanget = {180, 135, 225}; // 180 for Inner 225 for Mid and Outer FROM INNER// 135 for FROM OUTER
    protected static Integer[] parkingTanget = {225, 135}; // 135 for Inner 225 for Mid and Outer

    protected static Integer leftSpikeStartingTangetValue = 0;
    protected static Integer leftSpikeFinalTangetValue = 0;
    protected static Integer stackStationTangetValue = 0;
    protected static Integer parkingTangetValue = 1;

    protected static Pose2d leftPixel_SHORT = new Pose2d(Tile, Tile + (RobotX/2), Math.toRadians(0));
    protected static Pose2d centerPixel_SHORT = new Pose2d(Tile/2, Tile + (RobotY/2), Math.toRadians(270));
    protected static Pose2d rightPixel_SHORT = new Pose2d((RobotY/2), 1.5 * Tile, Math.toRadians(180));

    protected static Pose2d leftPixel_LONG = new Pose2d(2 * TileInverted,1.5 * Tile, Math.toRadians(180));
    protected static Pose2d centerPixel_LONG = new Pose2d(1.5 * TileInverted, Tile + (RobotY/2), Math.toRadians(270));
    protected static Pose2d rightPixel_LONG = new Pose2d(TileInverted - (RobotY/2), Tile + (RobotX/2), Math.toRadians(0));

    protected static Pose2d backdropLeft = new Pose2d(2.5 * Tile - (RobotY/2), 1.25 * Tile, Math.toRadians(180)); // Default
    protected static Pose2d backdropCenter = new Pose2d(2.5 * Tile - (RobotY/2), 1.5 * Tile, Math.toRadians(180));
    protected static Pose2d backdropRight = new Pose2d(2.5 * Tile - (RobotY/2), 1.75 * Tile, Math.toRadians(180)); // Default

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

    public static Pose2d pixel_cycle_PoseTransfer = rightPixel_SHORT;
    public static Pose2d leftPixelSpike = leftPixel_SHORT;
    public static Pose2d centerPixelSpike = centerPixel_SHORT;
    public static Pose2d rightPixelSpike = rightPixel_SHORT;
    public static Pose2d randomizedBackdrop = backdropRight;
    public static Vector2d stationClose = stationClose_Inner;
    public static Vector2d stationFar = stationFar_Inner;
    public static Pose2d backdrop_Unload = backdropLeft;
    public static Pose2d stackStation = stationInner;
    public static Pose2d parkingPose = parkingMiddle;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        randomizedBackdrop = backdropLeft;
        stationClose = stationClose_Inner;
        stationFar = stationFar_Inner;
        backdrop_Unload = backdropLeft;
        stackStation = stationInner;
        leftSpikeStartingTangetValue = 0;
        leftSpikeFinalTangetValue = 0;
        stackStationTangetValue = 0;
        leftPixelSpike = leftPixel_SHORT;
        centerPixelSpike = centerPixel_SHORT;
        rightPixelSpike = rightPixel_SHORT;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(320), Math.toRadians(320), 9.4)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(HomePose_SHORT)

//                .addDisplacementMarker(() -> {
////                    randomizationPixelElevator();

                                .setTangent(Math.toRadians(leftSpikeStartingTanget[leftSpikeStartingTangetValue])) //tan pair 45/135
                                .splineToLinearHeading(rightPixelSpike, Math.toRadians(leftSpikeFinalTanget[leftSpikeFinalTangetValue]))

//                                .strafeTo(leftPixelSpike.vec())
//                })
                                .setTangent(Math.toRadians(90))
                                .splineToLinearHeading(randomizedBackdrop, Math.toRadians(300))
//                .addDisplacementMarker(() -> {
////                    scoring();
//                })
                                /*-------------------------------------------------------------------*/
                                /*----2+2----*/
                                /*-------------------------------------------------------------------*/
                                .setTangent(Math.toRadians(180))
                                .splineToConstantHeading(stationClose, Math.toRadians(180))
                                .lineTo(stationFar)
                                .splineToConstantHeading(stackStation.vec(), Math.toRadians(stackStationTanget[stackStationTangetValue])) //tan pair 180/225
//                .addDisplacementMarker(() -> {
////                    stackStationIntake();
//                })
                                .setReversed(true)
                                .setTangent(Math.toRadians(0))
                                .splineToConstantHeading(stationFar, Math.toRadians(0))
                                .lineTo(stationClose)
//                .addDisplacementMarker(() -> {
////                    elevator();
//                })
                                .splineToConstantHeading(backdrop_Unload.vec(), Math.toRadians(0))
//                .addDisplacementMarker(() -> {
////                    scoring();
//                })
                                /*-------------------------------------------------------------------*/
                                /*----2+4----*/
                                /*-------------------------------------------------------------------*/
                                .setTangent(Math.toRadians(180))
                                .splineToConstantHeading(stationClose, Math.toRadians(180))
                                .lineTo(stationFar)
                                .splineToConstantHeading(stackStation.vec(), Math.toRadians(stackStationTanget[stackStationTangetValue])) //tan pair 180/225
//                .addDisplacementMarker(() -> {
////                    stackStationIntake();
//                })
                                .setReversed(true)
                                .setTangent(Math.toRadians(0))
                                .splineToConstantHeading(stationFar, Math.toRadians(0))
                                .lineTo(stationClose)
//                .addDisplacementMarker(() -> {
////                    elevator();
//                })
                                .splineToConstantHeading(backdrop_Unload.vec(), Math.toRadians(0))
//                .addDisplacementMarker(() -> {
////                    scoring();
//                });
                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}