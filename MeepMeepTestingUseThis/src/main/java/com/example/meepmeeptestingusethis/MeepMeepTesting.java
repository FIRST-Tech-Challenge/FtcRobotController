package com.example.meepmeeptestingusethis;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();
        RoadRunnerBotEntity myBot2 = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-13.72, -61.32, Math.toRadians(90.0)))
        .splineToConstantHeading(new Vector2d(-9.71, -34.78), Math.toRadians(81.42))
                .waitSeconds(1)
                .strafeToConstantHeading(new Vector2d(-32, -39))
                .splineToSplineHeading(new Pose2d(-62, -53, Math.toRadians(45)), Math.toRadians(270))
                .waitSeconds(1)
                .turnTo(Math.toRadians(70))
                .waitSeconds(1)
                .turnTo(Math.toRadians(45))
                .waitSeconds(1)
                .turnTo(Math.toRadians(85))
                .waitSeconds(1)
                .turnTo(Math.toRadians(45))
                .waitSeconds(1)
                .turnTo(Math.toRadians(100))
                .waitSeconds(1)
                .turnTo(Math.toRadians(45))
                .splineTo(new Vector2d(-34, 0.07), Math.toRadians(0))
                .build());

        myBot2.runAction(myBot2.getDrive().actionBuilder(new Pose2d(12.09+3.5, -59.84, Math.toRadians(90.0)))
                .splineTo(new Vector2d(10,-38),Math.toRadians(90))
                .waitSeconds(1.5)
                .strafeToSplineHeading(new Vector2d(8,-39),Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-51,-53.5,Math.toRadians(0)),Math.toRadians(180))
                .strafeTo(new Vector2d(0,-52))
                .splineToSplineHeading(new Pose2d(47,-38,Math.toRadians(90)),Math.toRadians(180))
                .strafeTo(new Vector2d(0,-52))
                .splineToSplineHeading(new Pose2d(-51,-53.5,Math.toRadians(0)),Math.toRadians(180))
                .strafeTo(new Vector2d(0,-52))
                .splineToSplineHeading(new Pose2d(56,-38,Math.toRadians(90)),Math.toRadians(180))
                .strafeTo(new Vector2d(0,-52))
                .splineToSplineHeading(new Pose2d(-51,-53.5,Math.toRadians(0)),Math.toRadians(180))
                .strafeTo(new Vector2d(0,-52))
                .splineToSplineHeading(new Pose2d(56,-38,Math.toRadians(45)),Math.toRadians(180))
                .strafeTo(new Vector2d(0,-52))
                .splineToSplineHeading(new Pose2d(-51,-53.5,Math.toRadians(0)),Math.toRadians(180))
                .strafeTo(new Vector2d(56,-60))
                .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .addEntity(myBot2)
                .start();
    }
}