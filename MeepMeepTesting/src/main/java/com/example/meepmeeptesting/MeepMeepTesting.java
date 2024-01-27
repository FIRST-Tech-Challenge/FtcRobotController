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
                .setConstraints(39.75157189961097, 64.48221895918918, Math.toRadians(180), Math.toRadians(180), 12.29)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, -61, Math.toRadians(90)))
                                .splineTo(new Vector2d(7.5, -36), Math.toRadians(135))
                                .back(17.5)
                                .splineToLinearHeading(new Pose2d(44.25, -24, Math.toRadians(0)), Math.toRadians(0))
//                                .splineToLinearHeading(new Pose2d(45, -36, Math.toRadians(0)), Math.toRadians(0))
                                .forward(14)
                                .back(2.2)
                                .back(8)
                                .strafeRight(38)
                                .forward(15)
//                                .splineTo(new Vector2d(45, -36), Math.toRadians(0))
                                .build()
                );
        RoadRunnerBotEntity myBot4 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(39.75157189961097, 64.48221895918918, Math.toRadians(180), Math.toRadians(180), 12.29)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(12, -61, Math.toRadians(90)))
                                        .splineTo(new Vector2d(16.5, -36), Math.toRadians(45))
                                        .back(17.5)
                                        .splineToLinearHeading(new Pose2d(-20.25, -24, Math.toRadians(180)), Math.toRadians(180))
//                                .splineToLinearHeading(new Pose2d(45, -36, Math.toRadians(0)), Math.toRadians(0))
                                        .forward(14)
                                        .back(2.2)
                                        .back(8)
                                        .strafeLeft(38)
                                        .forward(15)
//                                .splineTo(new Vector2d(45, -36), Math.toRadians(0))
                                        .build()
                );

        RoadRunnerBotEntity myBot2 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(39.75157189961097, 64.48221895918918, Math.toRadians(180), Math.toRadians(180), 12.29)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(12, -61, Math.toRadians(90)))
                                        .lineToConstantHeading(new Vector2d(28.5, -36))
                                        .back(17.5)
                                        .splineToLinearHeading(new Pose2d(46, -33.5, Math.toRadians(0)), Math.toRadians(0))
                                        .forward(14)
//                                .splineToLinearHeading(new Pose2d(45, -36, Math.toRadians(0)), Math.toRadians(0))
                                        .back(2.45)
                                        .back(8)
                                        .strafeLeft(30)
                                        .forward(15)
//                                .splineTo(new Vector2d(45, -36), Math.toRadians(0))
                                        .build()
                );
        RoadRunnerBotEntity myBot5 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(39.75157189961097, 64.48221895918918, Math.toRadians(180), Math.toRadians(180), 12.29)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(12, -61, Math.toRadians(90)))
                                        .lineToConstantHeading(new Vector2d(-4.5, -36))
                                        .back(17.5)
                                        .splineToLinearHeading(new Pose2d(-22, -33.5, Math.toRadians(180)), Math.toRadians(180))
                                        .forward(14)
//                                .splineToLinearHeading(new Pose2d(45, -36, Math.toRadians(0)), Math.toRadians(0))
                                        .back(2.45)
                                        .back(8)
                                        .strafeLeft(30)
                                        .forward(15)
//                                .splineTo(new Vector2d(45, -36), Math.toRadians(0))
                                        .build()
                );

        RoadRunnerBotEntity myBot3 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(39.75157189961097, 64.48221895918918, Math.toRadians(180), Math.toRadians(180), 12.29)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(12, -61, Math.toRadians(90)))
                                        .lineTo(new Vector2d(12, -29))
                                        .back(17.5)
                                        .splineToLinearHeading(new Pose2d(45, -28.8, Math.toRadians(0)), Math.toRadians(0))
                                        .forward(14)
                                        .back(3.3)
                                        .back(8)
                                        .strafeRight(34)
                                        .forward(15)
                                        .build()
                );
        RoadRunnerBotEntity myBot6 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(39.75157189961097, 64.48221895918918, Math.toRadians(180), Math.toRadians(180), 12.29)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, -61, Math.toRadians(90)))
                                .lineTo(new Vector2d(12, -29))
                                .back(17.5)
                                .splineToLinearHeading(new Pose2d(-21, -28.8, Math.toRadians(180)), Math.toRadians(180))
                                .forward(14)
                                .back(3.3)
                                .back(8)
                                .strafeLeft(34)
                                .forward(15)
                                .build()
                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
//                .addEntity(myBot)
//                .addEntity(myBot2)
                .addEntity(myBot3)
//                .addEntity(myBot4)
//                .addEntity(myBot5)
                .addEntity(myBot6)
                .start();
    }
}