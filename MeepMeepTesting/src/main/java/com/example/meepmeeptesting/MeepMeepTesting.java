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
                .setDimensions(15,19)
                .setConstraints(100, 60, 4 * PI, 2 * PI, 16)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(17,-61, toRadians(-90)))
                                        .lineToLinearHeading(new Pose2d(24.5,-43, toRadians(-90)))
                                        .lineToLinearHeading(new Pose2d(46.4, -29, toRadians(-180)))
                                        .setReversed(false)
                                        .splineToConstantHeading(new Vector2d(5, -11), toRadians(180))
                                        .splineToConstantHeading(new Vector2d(-30, -11), toRadians(180))
                                        .splineToConstantHeading(new Vector2d(-55, -11), toRadians(180))
                                        .setReversed(true)
                                        .splineToConstantHeading(new Vector2d(-30, -11), toRadians(0))
                                        .splineToConstantHeading(new Vector2d(5, -11), toRadians(0))
                                        .splineToConstantHeading(new Vector2d(46.4, -29), toRadians(0))
                                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}