package com.atlantadroids.meepmeeptesting;

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
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 17)
                .setDimensions(17.8, 17)
                .setStartPose(new Pose2d(0, -61, Math.toRadians(180)))
                .build();

//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, -61, Math.toRadians(180)))
//                .strafeTo(new Vector2d(0, -30.5))
//                .build());

        // Prepare to push specimen
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, -30.5, Math.toRadians(180)))
                .strafeTo(new Vector2d(0, -40))
                .strafeToLinearHeading(new Vector2d(31, -40), Math.toRadians(0))
                .strafeTo(new Vector2d(38, -20))
                .build());
        // push samples
//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(40, -24, Math.toRadians(0)))
//                        .strafeTo(new Vector2d(40, -61))
//                .strafeTo(new Vector2d(45, -24))
//                .strafeTo(new Vector2d(45, -61)).build());

//        // push to prime pickup
//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(45, -61, Math.toRadians(0)))
//                .strafeTo(new Vector2d(43, -55)).build());

//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, -34, Math.toRadians(180)))
//                .strafeToLinearHeading(new Vector2d(40, -55), Math.toRadians(0))
//                .waitSeconds(1)
//                .strafeTo(new Vector2d(40, -60)).build());
//
//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(40, -60, 0))
//                .strafeToLinearHeading(new Vector2d(0, -34), Math.toRadians(180)).build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}