package com.example.meepmeeptesting;


import static java.lang.Math.PI;
import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class MeepMeepTesting {

    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(15,19)
                .setConstraints(100, 60, 4 * PI, 2 * PI, 16)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(-38 ,60,toRadians(90)))
                                        //purp
                                        .setReversed(true)
                                        .lineToLinearHeading(new Pose2d(25.5,-40, toRadians(-90)))
                                        .lineToLinearHeading(new Pose2d(25.5,-45, toRadians(-90)))


                                        //intake
                                        .setReversed(false)
                                        .lineToLinearHeading(new Pose2d(46.4, -38.25, toRadians(-180)))
                                        //drop
                                        .setReversed(false)
                                        .splineToConstantHeading(new Vector2d(17, -11.25), toRadians(180))
                                        .splineToConstantHeading(new Vector2d(-30, -11.25), toRadians(180))
                                        .splineToConstantHeading(new Vector2d(-52, -11.25), toRadians(180))
                                        //pickup
                                        .setReversed(true)
                                        .splineToConstantHeading(new Vector2d(-30, -11.25), toRadians(0))
                                        .splineToConstantHeading(new Vector2d(5, -11.25), toRadians(0))
                                        .splineToConstantHeading(new Vector2d(49.4, -33), toRadians(0))

                                        .setReversed(false)
                                .splineToConstantHeading(new Vector2d(17, -11.25), toRadians(180))
                                .splineToConstantHeading(new Vector2d(0, -11.25), toRadians(180))
                                .splineToSplineHeading(new Pose2d(-53.5, -17.25, toRadians(200)), toRadians(200))
                                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}