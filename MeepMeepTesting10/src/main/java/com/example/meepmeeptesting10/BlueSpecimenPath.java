package com.example.meepmeeptesting10;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class BlueSpecimenPath {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width

                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        TrajectoryActionBuilder tab1 = myBot.getDrive().actionBuilder(
//                new Pose2d(11.8, 61.7, Math.toRadians(90)))
                new Pose2d(-24, 60, Math.toRadians(-90)))
//                .lineToYSplineHeading(33, Math.toRadians(0))
//                .waitSeconds(2)
                .setTangent(Math.toRadians(90))
//                .lineToY(48)
//                .setTangent(Math.toRadians(90))
//                .lineToX(-32)
                .strafeTo(new Vector2d(-44.5, 30))
                .strafeTo(new Vector2d(-24, 50))
                .strafeTo(new Vector2d(50, 50))
                .turnTo(45)
                .waitSeconds(3)
                .strafeToLinearHeading(
                        new Vector2d(-24, 50), 90
                )
//                .strafeTo(new Vector2d(-24, 50))
                .turnTo(90)
                .strafeTo(new Vector2d(-44.5, 30))
//                .splineTo(new Vector2d(44.5, 30), -45)
//                .turn(Math.toRadians(180))
//                .lineToX(47.5)
                .waitSeconds(3);

        Pose2d initialPose = new Pose2d(-14, 60, Math.toRadians(-90));

        // Go till the first sample
        TrajectoryActionBuilder sample1PickSampleTrajectory = myBot.getDrive().actionBuilder(
                        initialPose)
                .waitSeconds(1)
                .strafeTo(new Vector2d(0, 38))
//                        new Pose2d(-24, 60, Math.toRadians(-90)))
                // Spline to the first sample and turn towards it
//                .splineTo(new Vector2d(-38, 24.0), 180)
//                .splineToSplineHeading(new Pose2d(-30.0, 20.0, Math.PI), 0)
//                .turnTo(-135.0)
//                .waitSeconds(2)
                .strafeTo(new Vector2d(-48.0, 38.0))
//                .waitSeconds(1)
                .strafeTo(new Vector2d(0, 38))
//                .waitSeconds(2)
                .strafeTo(new Vector2d(-57.0, 38.0))
//                .waitSeconds(1)
                .strafeTo(new Vector2d(0, 38))
//                .waitSeconds(2)
                .strafeTo(new Vector2d(-57.0, 38.0))
                .turnTo(Math.toRadians(-180))
                .strafeTo(new Vector2d(-57.0, 24))
//                .waitSeconds(1)
                .strafeTo(new Vector2d(-56, 38))
                .turnTo(Math.toRadians(-90))
                .strafeTo(new Vector2d(0, 38))
                .waitSeconds(2);


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

        myBot.runAction(
                // Go to sample 1
                new SequentialAction(sample1PickSampleTrajectory.build(),
                        // Pick up sample 1
                        // Claw code
                        // Go to blue basket
                        new SequentialAction(sample1PickSampleTrajectory.build(),
                                // Drop the sample in the upper blue basket
                                new SequentialAction(sample2PickSampleTrajectory.build())
        )));
    }
}