package com.example.meepmeeptesting;


import static java.lang.Math.PI;
import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class MeepMeepTesting {

    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(16.5,17.5)
                .setConstraints(100, 60, 4 * PI, 2 * PI, 16)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(-38.5, -62, Math.toRadians(-90)))
                                        .setReversed(true)
                                        .lineToLinearHeading(new Pose2d(-55, -32, toRadians(-160)))
                                        .setReversed(true)
                                        .lineToLinearHeading(new Pose2d(-40, -58, toRadians(180)))
                                        .setReversed(true)
                                        .splineTo(new Vector2d(5, -57.5), toRadians(10))
                                        .splineTo(new Vector2d(52.5, -28), toRadians(0))
                                        .setReversed(false)
                                        .splineTo(new Vector2d(5, -57.5), toRadians(185))
                                        .splineTo(new Vector2d(-20, -58), toRadians(175))
                                        .splineTo(new Vector2d(-57, -37.5), toRadians(165))
                                        .setReversed(true)
                                        .splineTo(new Vector2d(-22, -57.5), toRadians(-10))
                                        .splineTo(new Vector2d(5, -57.5), toRadians(10))
                                        .splineTo(new Vector2d(52.5, -44), toRadians(0))
                                        .setReversed(false)
                                        .splineTo(new Vector2d(5, -57.5), toRadians(185))
                                        .splineTo(new Vector2d(-20, -58), toRadians(175))
                                        .splineTo(new Vector2d(-57, -37.5), toRadians(165))
                                        .setReversed(true)
                                        .splineTo(new Vector2d(-22, -57.5), toRadians(-10))
                                        .splineTo(new Vector2d(5, -57.5), toRadians(10))
                                        .splineTo(new Vector2d(52.5, -44), toRadians(0))
                                        .setReversed(false)
                                        .splineTo(new Vector2d(5, -57.5), toRadians(185))
                                        .splineTo(new Vector2d(-20, -58), toRadians(175))
                                        .splineTo(new Vector2d(-57, -28), toRadians(155))
                                        .setReversed(true)
                                        .splineTo(new Vector2d(-22, -57.5), toRadians(-10))
                                        .splineTo(new Vector2d(5, -57.5), toRadians(10))
                                        .splineTo(new Vector2d(52.5, -44), toRadians(0))
                                        .build()


                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}