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
                                drive.trajectorySequenceBuilder(new Pose2d(-36,-64, toRadians(-90)))
                                        //purp
                                        .setReversed(true)
                                        .splineToLinearHeading(new Pose2d(-44, -35.25, toRadians(-90)), toRadians(90))
                                        //intake
                                        .setReversed(false)
                                        .splineToLinearHeading(new Pose2d(-46, -45.25, toRadians(220)), toRadians(-160))
                                        .lineToLinearHeading(new Pose2d(-52, -35.25, toRadians(180)))
                                        //drop
                                        .setReversed(true)
                                        .splineToConstantHeading(new Vector2d(-35,-58),toRadians(0))
                                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(30,5,15))
                                        .splineToConstantHeading(new Vector2d(-25,-58),toRadians(0))
                                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(80,5,15))
                                        .splineToConstantHeading(new Vector2d(5,-58),toRadians(0))
                                        .splineToConstantHeading(new Vector2d(44,-35.25),toRadians(0))
                                        //pickup
                                        .setReversed(false)
                                        .splineToConstantHeading(new Vector2d(25,-58),toRadians(180))
                                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(30,5,15))
                                        .splineToConstantHeading(new Vector2d(15,-58),toRadians(180))
                                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(80,5,15))
                                        .splineToConstantHeading(new Vector2d(-35,-58),toRadians(180))
                                        .splineToConstantHeading(new Vector2d(-52,-35.25),toRadians(180))
                                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}