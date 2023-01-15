package com.example.meepmeeptesting;


import static java.lang.Math.PI;
import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class MeepMeepTesting {
    public static double dropX = 32.2, dropY = 18.9 , dropA = toRadians(330), dropET = toRadians(150);

    public static double pickupX1 = -46, pickupY1 = 10, pickupA1 = toRadians(180), pickupET1 = toRadians(180);
    public static double pickupX2 = 64.75, pickupY2 = 11.75, pickupA2 = toRadians(0), pickupET2 = toRadians(180);

    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(600);


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(13.5,14.5)
                .setConstraints(50, 40, 4 * PI, 2 * PI, 11)
                .setConstraints(60, 60, toRadians(180), toRadians(180), 15)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(42.85, 63.25, Math.toRadians(90)))
                                        .setReversed(true)
//                                        .lineTo(new Vector2d(37.5,56))
//                                        .splineToSplineHeading(new Pose2d(32, 30.5, toRadians(45)), toRadians(270))
                                        .setReversed(true)
//                .splineTo(new Vector2d(34,50),toRadians(270))
//                .lineTo(new Vector2d(37.5,56))
//                .splineToSplineHeading(new Pose2d(32, 30.5, toRadians(45)), toRadians(270))
                                        .lineTo(new Vector2d(37,55.5))
                                        .splineToSplineHeading(new Pose2d(36, 30, toRadians(90)), toRadians(270))
//                                        .splineTo(new Vector2d(35.5, 13),toRadians(220))
                                        .splineToLinearHeading(new Pose2d(dropX,dropY,dropA),dropET)
//                                        .lineTo(new Vector2d(37,55.5))
//                                        .splineToSplineHeading(new Pose2d(33, 30, toRadians(90)), toRadians(270))
//                                        .splineTo(new Vector2d(35.5, 13),toRadians(220))
//                                        .splineToLinearHeading(new Pose2d(dropX,dropY,dropA),dropET)
                                        .setReversed(false)
//                                        .splineToLinearHeading(new Pose2d(36,11.5, toRadians(0)),toRadians(240))
                                        .splineToSplineHeading(new Pose2d(pickupX2-10, pickupY2+2,toRadians(0)), 0)
                                .splineToConstantHeading(new Vector2d(pickupX2, pickupY2), 0)
                                        .setReversed(true)
                                        .splineTo(new Vector2d(dropX,dropY), dropET)
//                                        .setReversed(false)
////                                        .splineToSplineHeading(new Pose2d(69,69,toRadians(69)),toRadians(69))
////                                        .splineToSplineHeading(new Pose2d(4200,420,toRadians(420)),toRadians(420))
////                                        .splineToSplineHeading(new Pose2d(69,420,toRadians(69)),toRadians(420))
////                                        .splineToSplineHeading(new Pose2d(420,69,toRadians(420)),toRadians(69))
//
//                                        .splineToSplineHeading(new Pose2d(-50, 10, toRadians(180)), toRadians(180))
//                                        .setReversed(true)
//                                        .lineTo(new Vector2d(-10, 10.5))
//                                        .setReversed(false)
//                                        .splineToSplineHeading(new Pose2d(pickupX2, pickupY2, pickupA2), toRadians(360))
//                                        .setReversed(true)
//                                        .splineToSplineHeading(new Pose2d(dropX,dropY, dropA), dropET)
//                                        .setReversed(false)
//                                        .splineToSplineHeading(new Pose2d(58, 12, toRadians(360)), toRadians(360))

                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}