package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(45, 60, Math.toRadians(60), Math.toRadians(60), 16.4)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-15, 44, Math.toRadians(270)))
                                .lineToSplineHeading(new Pose2d(new Vector2d(5, 60), Math.toRadians(180)))
                                .splineToLinearHeading(new Pose2d( new Vector2d(48, 66), Math.toRadians(180)), Math.toRadians(0))
                                .lineTo(new Vector2d(15, 66))
                                .splineToSplineHeading(new Pose2d(-15, 44, Math.toRadians(270)), Math.toRadians(270))
                                //.lineToSplineHeading(new Pose2d(new Vector2d(-15, -44), Math.toRadians(90)))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}