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
                .setConstraints(80, 40, 4 * PI, 2 * PI, 16)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(-50,-31.25, toRadians(-180)))
                                        .setReversed(true)
                                        .splineToConstantHeading(new Vector2d(-28, -58.5), toRadians(3))
                                        .splineToConstantHeading(new Vector2d(0, -58.5), toRadians(0))
                                        .splineTo(new Vector2d(46.3, -35), toRadians(0))
                                        .setReversed(false)
                                        .splineTo(new Vector2d(3, -58.5), toRadians(180))
                                        .splineToConstantHeading(new Vector2d(-28, -58.5), toRadians(180))
                                        .splineToConstantHeading(new Vector2d(-53, -35.25), toRadians(150))

                                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}