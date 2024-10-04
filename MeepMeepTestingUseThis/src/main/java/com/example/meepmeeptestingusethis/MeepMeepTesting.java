package com.example.meepmeeptestingusethis;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-13.72, -61.32, Math.toRadians(90.00)))
                .splineToConstantHeading(new Vector2d(-9.71, -34.78), Math.toRadians(81.42))
                .waitSeconds(2)
                .strafeToConstantHeading(new Vector2d(-32,-39))
                .splineToSplineHeading(new Pose2d(-58, -53,Math.toRadians(45)), Math.toRadians(270))
                .waitSeconds(2)
                .turnTo(Math.toRadians(70))
                .waitSeconds(2)
                .turnTo(Math.toRadians(45))
                .waitSeconds(2)
                .turnTo(Math.toRadians(85))
                .waitSeconds(2)
                .turnTo(Math.toRadians(45))
                .waitSeconds(2)
                .turnTo(Math.toRadians(100))
                .waitSeconds(2)
                .turnTo(Math.toRadians(45))
                .splineTo(new Vector2d(-34, 0.07), Math.toRadians(0))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}