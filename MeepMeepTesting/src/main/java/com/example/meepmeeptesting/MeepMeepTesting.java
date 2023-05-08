package com.example.meepmeeptesting;


import static java.lang.Math.PI;
import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class MeepMeepTesting {

    public static double dropX = -28.4, dropY = 4.3;

    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(600);


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(13.5,14.5)
                .setConstraints(50, 40, 4 * PI, 2 * PI, 11)
                .setConstraints(60, 60, toRadians(180), toRadians(180), 15)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(-34.5, 63.25, toRadians(90)))
                                        .setReversed(true)
                                        .splineToSplineHeading(new Pose2d(-34.5, 40.25, toRadians(90)), toRadians(270))
                                        .splineTo(new Vector2d(-33.5, 22), toRadians(275))
                                        .splineToSplineHeading(new Pose2d(-27, 4.5, toRadians(140)), toRadians(320))

                                        .setReversed(false)
                                        .splineTo(new Vector2d(-63.5, 11.5), Math.toRadians(180))

                                        .setReversed(true)
                                        .splineToSplineHeading(new Pose2d(dropX, dropY, Math.toRadians(145)), Math.toRadians(325))

                                        .setReversed(false)
                                        .splineTo(new Vector2d(-63.5, 10.5), Math.toRadians(180))

                                        .setReversed(true)
                                        .splineToSplineHeading(new Pose2d(dropX, dropY, Math.toRadians(145)), Math.toRadians(325))

                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}