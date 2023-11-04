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
                                drive.trajectorySequenceBuilder(new Pose2d(15.5, 63.25, toRadians(90)))

                                        .setReversed(true) //spike 1
                                        .splineToLinearHeading(new Pose2d(8, 37, toRadians(45)), toRadians(225))
                                        .lineToConstantHeading(new Vector2d(13, 42))
                                        .setReversed(false)
                                        .lineToLinearHeading(new Pose2d(56, 30.5, toRadians(180)))
                                        .lineToLinearHeading(new Pose2d(56, 60, toRadians(180)))
                                        .lineToLinearHeading(new Pose2d(60, 60, toRadians(180)))
                                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}