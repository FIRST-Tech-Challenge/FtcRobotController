package com.example.meepmeeptesting;


import static java.lang.Math.PI;
import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class MeepMeepTesting {
    public static double dummyP = 3;

    public static double dummyxi = -12.5, dummyyi = 55;
    public static double dummyxi2 = -12.5, dummyyi2 = 13;


    public static double dummyx = -24.5, dummyy = 4.5, dummya = 270;
    public static double dummyx2 = -24.5, dummyy2 =11, dummya2 = 270;
    public static double dummyxd = -24.5, dummyyd = 5, dummyad = 270;
    public static double dummyx2i = -24.5, dummyy2i =11, dummya2i = 270;
    public static double dummyx3i = -24.5, dummyy3i =9, dummya3i = 270;
    public static double dummyx3 = -38, dummyy3 =10.1, dummya3 = 180;
    public static double dummyx4 = -63.5, dummyy4 =10.1, dummya4 = 180;

    public static double dummyX = -12, dummyY = 11, dummyA = 180;

    public static double dummyX2 = -35, dummyY2 = 11, dummyA2 = 180;

    public static double dummyX3 = -55, dummyY3 = 11, dummyA3 = 180;
    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(700);


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(13.5,14.5)
                .setConstraints(50, 40, 4 * PI, 2 * PI, 11)
                .setConstraints(60, 60, toRadians(180), toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-29.6, 62.25, toRadians(90)))
                                .setReversed(true).splineToSplineHeading(new Pose2d(-35, 40, Math.toRadians(85)), Math.toRadians(265))
                                .splineToSplineHeading(new Pose2d(-35, 30, Math.toRadians(100)), Math.toRadians(280))
                                .splineToSplineHeading(new Pose2d(-28.5,7.1,Math.toRadians(120)),Math.toRadians(120))
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(-45, 11.75, Math.toRadians(180)), Math.toRadians(180))
                                .splineToSplineHeading(new Pose2d(-60.5, 11.75, Math.toRadians(180)), Math.toRadians(180))
                                /*.setReversed(false)
                                .splineToSplineHeading(new Pose2d(-63.5,11.75,Math.toRadians(180)),Math.toRadians(180))
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(-30,5.5,Math.toRadians(140)),Math.toRadians(320))
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(-63.5,11.75,Math.toRadians(180)),Math.toRadians(180))
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(-30,5.5,Math.toRadians(140)),Math.toRadians(320))
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(-63.5,11.75,Math.toRadians(180)),Math.toRadians(180))
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(-30,5.5,Math.toRadians(140)),Math.toRadians(320))                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(-63.5,11.75,Math.toRadians(180)),Math.toRadians(180))
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(-30,5.5,Math.toRadians(140)),Math.toRadians(320))                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(-63.5,11.75,Math.toRadians(180)),Math.toRadians(180))
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(-30,5.5,Math.toRadians(140)),Math.toRadians(320))                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(-36, 33,Math.toRadians(90)),Math.toRadians(90))*/
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}