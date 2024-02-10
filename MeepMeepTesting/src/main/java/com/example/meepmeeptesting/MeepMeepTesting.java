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
                                drive.trajectorySequenceBuilder(new Pose2d(-38, -61, Math.toRadians(-90)))
                                        .setReversed(true)
                                        .lineToLinearHeading(new Pose2d(-40,-38,toRadians(-90)))
                                        .lineToLinearHeading(new Pose2d(-38,-40,toRadians(-180)))
                                        .setReversed(false)
                                        .lineToLinearHeading(new Pose2d(-46,-58, toRadians(-180)))
                                        .setReversed(true)
                                        .lineToLinearHeading(new Pose2d(20,-58, toRadians(-180)))
                                        .lineToLinearHeading(new Pose2d(50,-30,toRadians(-180)))
                                        .lineTo(new Vector2d(50,-10))
                                        .build()


                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}