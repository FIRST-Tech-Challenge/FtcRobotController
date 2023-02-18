package com.example.meepmeeptesting;


import static java.lang.Math.PI;
import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class MeepMeepTesting2 {

    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(700);


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(13.5,14.5)
                .setConstraints(50, 40, 4 * PI, 2 * PI, 11)
                .setConstraints(60, 60, toRadians(180), toRadians(180), 15)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(42, 63.25, toRadians(90)))

                                        .setReversed(true)
                                        .splineToSplineHeading(new Pose2d(39.6, 51, toRadians(70)), toRadians(250))
                                        .splineToSplineHeading(new Pose2d(37.6, 9, toRadians(315)), toRadians(270))
                                        .lineToLinearHeading(new Pose2d(27.8,17.25, toRadians(315)))

                                        .setReversed(false)
                                        .splineToSplineHeading(new Pose2d(44,12.5,toRadians(0)),toRadians(0))
                                        .splineToSplineHeading(new Pose2d(63.25+1.5-0.5,13-0.5,toRadians(0)),toRadians(0))

                                        .setReversed(true)
                                        .splineToSplineHeading(new Pose2d(29.5,20,toRadians(330)), toRadians(150))

                                        .setReversed(false)
                                        .splineToSplineHeading(new Pose2d(50,12.5,toRadians(0)),toRadians(0))
                                        .splineToSplineHeading(new Pose2d(63.25+1.5-0.5,13-0.5,toRadians(0)),toRadians(0))

                                        //preload
                                        .setReversed(true).splineToSplineHeading(new Pose2d(38, 51, toRadians(70)), toRadians(250))
                                        .splineToSplineHeading(new Pose2d(36, 17, toRadians(90)), toRadians(270))
                                        .splineToSplineHeading(new Pose2d(31.5,10, toRadians(50)), toRadians(215))

                                        //first cycle
                                        .setReversed(false)
                                        .splineToSplineHeading(new Pose2d(45, 12, Math.toRadians(0)), Math.toRadians(0))
                                        .splineToSplineHeading(new Pose2d(62.5, 12, Math.toRadians(0)), Math.toRadians(0))
                                        .setReversed(true)
                                        .splineToSplineHeading(new Pose2d(33.5, 8, Math.toRadians(40)), Math.toRadians(220))


                                        //cycling
                                        .setReversed(false)
                                        .splineToSplineHeading(new Pose2d(45, 12, Math.toRadians(0)), Math.toRadians(0))
                                        .splineToSplineHeading(new Pose2d(62.5, 12, Math.toRadians(0)), Math.toRadians(0))
                                        .setReversed(true)
                                        .splineToSplineHeading(new Pose2d(32, 5, Math.toRadians(40)), Math.toRadians(220))

                                        .setReversed(false)
                                        .splineToSplineHeading(new Pose2d(45, 12, Math.toRadians(0)), Math.toRadians(0))
                                        .splineToSplineHeading(new Pose2d(62.5, 12, Math.toRadians(0)), Math.toRadians(0))
                                        .setReversed(true)
                                        .splineToSplineHeading(new Pose2d(32, 5, Math.toRadians(40)), Math.toRadians(220))

                                        .setReversed(false)
                                        .splineToSplineHeading(new Pose2d(45, 12, Math.toRadians(0)), Math.toRadians(0))
                                        .splineToSplineHeading(new Pose2d(62.5, 12, Math.toRadians(0)), Math.toRadians(0))
                                        .setReversed(true)
                                        .splineToSplineHeading(new Pose2d(32, 5, Math.toRadians(40)), Math.toRadians(220))

                                        .setReversed(false)
                                        .splineToSplineHeading(new Pose2d(45, 12, Math.toRadians(0)), Math.toRadians(0))
                                        .splineToSplineHeading(new Pose2d(62.5, 12, Math.toRadians(0)), Math.toRadians(0))
                                        .setReversed(true)
                                        .splineToSplineHeading(new Pose2d(32, 5, Math.toRadians(40)), Math.toRadians(220))

//                                //park 1
//                                .setReversed(false)
//                                .splineToSplineHeading(new Pose2d(58, 12, Math.toRadians(0)), Math.toRadians(0))

//                                //park 2
//                                .setReversed(false)
//                                .splineToSplineHeading(new Pose2d(34, 12, Math.toRadians(90)), Math.toRadians(90))

                                        //park 3
                                        .setReversed(false)
                                        .splineToConstantHeading(new Vector2d(35, 7), Math.toRadians(130))
                                        .splineTo(new Vector2d(10, 12), Math.toRadians(180))

//                                .splineToSplineHeading(new Pose2d(-63.5,11.75,Math.toRadians(180)),Math.todRadians(180))
//                                .setReversed(true)
//                                .splineToSplineHeading(new Pose2d(-30,5.5,Math.toRadians(140)),Math.toRadians(320))
//                                .setReversed(false)
//                                .splineToSplineHeading(new Pose2d(-63.5,11.75,Math.toRadians(180)),Math.toRadians(180))
//                                .setReversed(true)
//                                .splineToSplineHeading(new Pose2d(-30,5.5,Math.toRadians(140)),Math.toRadians(320))
//                                .setReversed(false)
//                                .splineToSplineHeading(new Pose2d(-63.5,11.75,Math.toRadians(180)),Math.toRadians(180))
//                                .setReversed(true)
//                                .splineToSplineHeading(new Pose2d(-30,5.5,Math.toRadians(140)),Math.toRadians(320))                                .setReversed(false)
//                                .splineToSplineHeading(new Pose2d(-63.5,11.75,Math.toRadians(180)),Math.toRadians(180))
//                                .setReversed(true)
//                                .splineToSplineHeading(new Pose2d(-30,5.5,Math.toRadians(140)),Math.toRadians(320))                                .setReversed(false)
//                                .splineToSplineHeading(new Pose2d(-63.5,11.75,Math.toRadians(180)),Math.toRadians(180))
//                                .setReversed(true)
//                                .splineToSplineHeading(new Pose2d(-30,5.5,Math.toRadians(140)),Math.toRadians(320))                                .setReversed(false)
//                                .splineToSplineHeading(new Pose2d(-36, 33,Math.toRadians(90)),Math.toRadians(90))*/
                                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}