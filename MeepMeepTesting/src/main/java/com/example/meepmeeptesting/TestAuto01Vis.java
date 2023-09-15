package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class TestAuto01Vis {
    enum autos {
        BBHIGH,
        BRHIGH,
        RBHIGH,
        RRHIGH,
        TESTING,
        SMOLTESTING
    };
    public static void main(String[] args) {
        System.setProperty("sun.java2d.opengl", "true");
        MeepMeep meepMeep = new MeepMeep(800);

        autos auto = autos.SMOLTESTING;
        RoadRunnerBotEntity myBot;
                myBot = new DefaultBotBuilder(meepMeep)
                        .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                        .followTrajectorySequence(drive ->
                                    drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                                            .strafeRight(20)
                                            .forward(50)
                                            .strafeLeft(40)
                                            .back(50)
                                            .strafeRight(20)
                                            .splineTo(new Vector2d(12,20),Math.toRadians(0))
                                            .splineTo(new Vector2d(11,20),Math.toRadians(69))
                                            .splineToConstantHeading(new Vector2d(40,0),Math.toRadians(69))
                                            .splineTo(new Vector2d(11, -34), Math.toRadians(123))
                                            .strafeRight(20)
                                            .splineTo(new Vector2d(50, 50), Math.toRadians(20))
                                            .splineTo(new Vector2d(-50, 50), Math.toRadians(20))
                                            .splineTo(new Vector2d(-50, -50), Math.toRadians(20))
                                            .splineTo(new Vector2d(50, -50), Math.toRadians(20))
                                            .splineTo(new Vector2d(0, 0), Math.toRadians(0))
                                            .build()
                        );
        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}