package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d startPos = new Pose2d (-40, -62, Math.PI/2);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
            .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
            .followTrajectorySequence(drive ->
                drive.trajectorySequenceBuilder(startPos)
                        .lineTo(new Vector2d(-36, -60))
                        .turn(Math.toRadians(-90))
                        .forward(24)
                        .turn(Math.toRadians(90))
                        .forward(24)
                        .turn(Math.toRadians(-45))
                        .forward(5)
                        .waitSeconds(2)
                        .forward(-5)
                        .turn(Math.toRadians(45))
                        .forward(24)
                        .turn(Math.toRadians(90))
                        .forward(48)
                        .waitSeconds(1)
                        .forward(-12)
                        .turn(Math.toRadians(90))
                        .waitSeconds(1)
                        .turn(Math.toRadians(-90))
                        .forward(12)
                        .waitSeconds(1)
                        .forward(-12)
                        .turn(Math.toRadians(90))
                        .waitSeconds(1)
                        .turn(Math.toRadians(-90))
                        .forward(12)
                        .waitSeconds(1)
                        .forward(-12)
                        .turn(Math.toRadians(90))
                        .waitSeconds(1)
                        .turn(Math.toRadians(-90))
                        .forward(12)
                        .waitSeconds(1)
                        .forward(-12)
                        .turn(Math.toRadians(90))
                        .waitSeconds(1)
                        .turn(Math.toRadians(-90))
                        .forward(12)
                        .waitSeconds(1)
                        .forward(-12)
                        .turn(Math.toRadians(90))
                        .waitSeconds(1)
                        .turn(Math.toRadians(-90))
                        .forward(12)
                        .build()
            );
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width


        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}