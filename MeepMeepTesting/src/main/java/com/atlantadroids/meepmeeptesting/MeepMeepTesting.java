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
                .setConstraints(60, 80, Math.toRadians(270), Math.toRadians(270), 17)
                .setDimensions(17.8, 17)
                .setStartPose(new Pose2d(0, -61, Math.toRadians(180)))
                .build();

//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, -61, Math.toRadians(180)))
//                 .strafeTo(new Vector2d(0, -31))
//                .build());

        // Prepare to push specimen
//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, -31, Math.toRadians(180)))
//                .splineToConstantHeading(new Vector2d(0,-39), Math.toRadians(0))
//                .splineToSplineHeading(new Pose2d(23.5,-38, Math.toRadians(50)), Math.toRadians(50))
//
//                .build());

        // push 1 sample
//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(23.5, -38, Math.toRadians(35)))
//            .strafeToLinearHeading(new Vector2d(23.5,-50), Math.toRadians(330))
//            .build()
//        );

        // push second sample
//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(23.5,-50, Math.toRadians(330)))
//            .strafeToLinearHeading(new Vector2d(32.5, -38), Math.toRadians(35))
//            .build()
//        );

        // go to third sample
//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(32.5, -50, Math.toRadians(330)))
//            .strafeToLinearHeading(new Vector2d(33, -20), Math.toRadians(0))
//            .build()
//        );

        // score second specimen
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(39, -65, Math.toRadians(0)))
//            .setReversed(true)
            .strafeToSplineHeading(new Vector2d(6, -31), Math.toRadians(180))
            .build()
        );

//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(6, -31, Math.toRadians(180)))
//            .strafeToSplineHeading(new Vector2d(39, -65), Math.toRadians(0))
//            .build()
//        );

        // deposit preload sample
//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-38, -61, Math.toRadians(90)))
//            .strafeToLinearHeading(new Vector2d(-56, -56), Math.toRadians(45))
//            .build());

        // go to first sample
//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-56, -56, Math.toRadians(45)))
//                .strafeToLinearHeading(new Vector2d(-31, -35), Math.toRadians(150))
//                    .build());

        // go to second sample
//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-56, -56, Math.toRadians(45)))
//        .strafeToLinearHeading(new Vector2d(-40, -35), Math.toRadians(150))
//            .build());

         myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-56, -56, Math.toRadians(45)))
        .strafeToLinearHeading(new Vector2d(-48, -25), Math.toRadians(180))
            .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}