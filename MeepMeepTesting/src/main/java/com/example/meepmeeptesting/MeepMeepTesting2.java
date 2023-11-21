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
                .setDimensions(16.5,17.5)
                .setConstraints(120, 60, 4 * PI, 2 * PI, 16)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(15.5, -63.25, toRadians(-90)))

                                        .setReversed(true) //spike 1
                                        .splineToLinearHeading(new Pose2d(8, -37, toRadians(-45)), toRadians(135))
                                        .lineToLinearHeading(new Pose2d(53, -30.5, toRadians(180)))
                                        .setReversed(false)
                                        .splineTo(new Vector2d(-10,-12), toRadians(180))
                                        .splineTo(new Vector2d(-58,-12), toRadians(180))
                                        .setReversed(true) //spike 1
                                        .splineTo(new Vector2d(-10,-12), toRadians(0))
                                        .splineTo(new Vector2d(53, -28.5), toRadians(0))
                                        .setReversed(false)
                                        .splineTo(new Vector2d(-10,-12), toRadians(180))
                                        .splineTo(new Vector2d(-58,-12), toRadians(180))
                                        .setReversed(true) //spike 1
                                        .splineTo(new Vector2d(-10,-12), toRadians(0))
                                        .splineTo(new Vector2d(53, -28.5), toRadians(0))
                                        .setReversed(false)
                                        .splineTo(new Vector2d(-10,-12), toRadians(180))
                                        .splineTo(new Vector2d(-58,-20), toRadians(190))
                                        .setReversed(true) //spike 1
                                        .splineTo(new Vector2d(-10,-12), toRadians(0))
                                        .splineTo(new Vector2d(53, -28.5), toRadians(0))
                                        .setReversed(false)
                                        .splineTo(new Vector2d(-10,-12), toRadians(180))
                                        .splineTo(new Vector2d(-58,-20), toRadians(190))
                                        .setReversed(true) //spike 1
                                        .splineTo(new Vector2d(-10,-12), toRadians(0))
                                        .splineTo(new Vector2d(53, -28.5), toRadians(0))


                                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}