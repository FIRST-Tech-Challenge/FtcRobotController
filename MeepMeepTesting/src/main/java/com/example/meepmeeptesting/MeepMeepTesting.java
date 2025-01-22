package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 18)
                .setDimensions(18, 18)
                .build();
        RoadRunnerBotEntity myBot2 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180),18)
                .setDimensions(18, 18)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(6+(18.0/2.0) -30, -(72-(18.0/2.0)), Math.toRadians(90)))
                .splineToLinearHeading(new Pose2d(0, -24-9, Math.toRadians(90)), Math.toRadians(90))
                .waitSeconds(2)
                .lineToY(-33)
                .splineToLinearHeading(new Pose2d(0, -26-9, Math.toRadians(90)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-55,-55, Math.toRadians(45)), Math.toRadians(-90))
                .build());
        myBot2.runAction(myBot.getDrive().actionBuilder(new Pose2d(6+(18.0/2.0), -(72-(18.0/2.0)), Math.toRadians(90)))
                        .splineToLinearHeading(new Pose2d(0, -20-9, Math.toRadians(90)), Math.toRadians(90))
                //armActions
//                .splineToLinearHeading(new Pose2d(22, -45, Math.toRadians(45)), Math.toRadians(90))
//                //extend
//                .turnTo(Math.toRadians(345))
//                //arm up little
//                .splineToLinearHeading(new Pose2d(32, -42, Math.toRadians(45)), Math.toRadians(-90))
//                //arm down
//                .turnTo(Math.toRadians(345))
//                //arm up little
//                .splineToLinearHeading(new Pose2d(40, -42, Math.toRadians(45)), Math.toRadians(-90))
//                //arm down
//                .turnTo(345)
//                .turnTo(Math.toRadians(-90))
//                //arm up
//                //extend less maybe
//                //intake
//                .waitSeconds(3)
//                .splineToLinearHeading(new Pose2d(6, -42, Math.toRadians(90)), Math.toRadians(0))
//                //arm up
//                //extend
//                //outtake
//                .strafeTo(new Vector2d(6, -43))
//                .splineToLinearHeading(new Pose2d(40, -45, Math.toRadians(-90)), Math.toRadians(0))
//                //arm up
//                //extend
//                //intake
//                .waitSeconds(1)
//                .splineToLinearHeading(new Pose2d(6, -40, Math.toRadians(90)), Math.toRadians(0))
//                //arm up
//                //extend
//                //outtake
//                .strafeTo(new Vector2d(6, -43))
//                .splineToLinearHeading(new Pose2d(40, -45, Math.toRadians(-90)), Math.toRadians(0))
//                //arm up
//                //extend
//                //intake
//                .waitSeconds(1)
//                .strafeTo(new Vector2d(6,-43))
//                .splineToLinearHeading(new Pose2d(6, -40, Math.toRadians(90)), Math.toRadians(0))
//                //arm up
//                //extend
//                //outtake
//                .splineToLinearHeading(new Pose2d(60,-60, Math.toRadians(-45)), Math.toRadians(0))
                .build());
        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
//                .addEntity(myBot2)
                .start();
    }
}