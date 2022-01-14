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
                        drive.trajectorySequenceBuilder(new Pose2d(-34, 63, Math.toRadians(270)))
                                .lineToSplineHeading(new Pose2d(new Vector2d(-12.5, 42), Math.toRadians(270)))
                                .lineToLinearHeading(new Pose2d( new Vector2d(-60, 60), Math.toRadians(0)))
                                .lineTo(new Vector2d(-55, 55))
                                .lineToLinearHeading(new Pose2d(-55, 54, Math.toRadians(270)))
                                .lineTo(new Vector2d(-55, 63))
                                .lineTo(new Vector2d(-50, 55))
                                .splineToLinearHeading(new Pose2d(-12.5, 42, Math.toRadians(270)), Math.toRadians(270))
                                .strafeRight(20)
                                //.lineTo(new Vector2d(-22, 44))
                                .splineToLinearHeading (new Pose2d(new Vector2d(-62, 35),Math.toRadians(270)), Math.toRadians(180))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}