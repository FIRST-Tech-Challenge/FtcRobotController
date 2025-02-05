package com.example.meepmeeptesting10;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class BluePushSampleDropToNetZoneTestPath {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width

                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        Pose2d initialPose = new Pose2d(-24, 60, Math.toRadians(-90));

        // Go till the first sample
        TrajectoryActionBuilder sample1PickSampleTrajectory = myBot.getDrive().actionBuilder(new Pose2d(24, 60, Math.toRadians(-90)))
               .waitSeconds(1)
//                .turnTo(Math.toRadians(90)
                .splineToLinearHeading(new Pose2d(48, 12, Math.toRadians(0)), 0)
                .waitSeconds(1)
                .strafeTo(new Vector2d(48, 48))
                .turnTo(Math.toRadians(-45))
                .waitSeconds(1)
                .strafeTo(new Vector2d(54, 54))
                .turnTo(Math.toRadians(0))
                .waitSeconds(1)
//                .strafeTo(new Vector2d(44, 12))
//                .splineToLinearHeading(new Pose2d(54, 12, Math.toRadians(-90)), -Math.PI)
//                .turnTo(Math.toRadians(45))
//                .strafeTo(new Vector2d(48, 48))
//                .turnTo(Math.toRadians(0))
//                .waitSeconds(1)
//                .strafeTo(new Vector2d(56, 24))
//                .waitSeconds(1)
//                .strafeTo(new Vector2d(48, 48))
//                .turnTo(Math.toRadians(45))
//                .strafeTo(new Vector2d(48, 48))
                .waitSeconds(1);


        // Grab the sample



        // Go to the blue basket
//        TrajectoryActionBuilder sample1DropSampleTrajectory = myBot.getDrive().actionBuilder(
//                        new Pose2d(-38, 24, Math.PI))
//                .waitSeconds(1)
////                .turnTo(90)
//                .strafeToLinearHeading(
//                        new Vector2d(-24, 50), 90
//                )
//                .strafeToLinearHeading(new Vector2d(50, 50), Math.PI/4);

        // Drop the sample into the upper bin

        // Go till the second sample
        TrajectoryActionBuilder sample2PickSampleTrajectory = myBot.getDrive().actionBuilder(
                        new Pose2d(50, 50, Math.PI/4))
                .waitSeconds(1)
//                        new Pose2d(-24, 60, Math.toRadians(-90)))
                // Spline to the first sample and turn towards it
//                .splineTo(new Vector2d(-38, 24.0), 180)
                .splineToSplineHeading(new Pose2d(-36.0, 24.0, Math.PI), 0)
//                .turnTo(-135.0)
                .waitSeconds(2);

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();

        myBot.runAction(new SequentialAction(myBot.getDrive().actionBuilder(
                        new Pose2d(24, 60, Math.toRadians(-90)))
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(46, 12, Math.toRadians(-90)), 0)
                .waitSeconds(1)
                .strafeTo(new Vector2d(48, 48))
                .turnTo(Math.toRadians(-135))
                .waitSeconds(1)
                .strafeTo(new Vector2d(54, 54))
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(54, 12, Math.toRadians(-90)), 0)
                .waitSeconds(1)
                .strafeTo(new Vector2d(54, 54))
                .turnTo(Math.toRadians(-135))
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(60, 12, Math.toRadians(-90)), 0)
                .strafeTo(new Vector2d(60, 54))
                .build()

        ));
    }
}