package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep mm = new MeepMeep(800);
        int detection = 3; // 1 = prop on left spike mark, 2 = middle, 3 = right
        int auto = 1;
        if (auto == 1) {
            if (detection == 1) {
                RoadRunnerBotEntity myBot = new DefaultBotBuilder(mm)
                        // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                        .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                        .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(-34, -60, Math.toRadians(90)))
                                        .lineToLinearHeading(new Pose2d(-34, -35, Math.toRadians(180)))
                                        .lineToLinearHeading(new Pose2d(38, -35, Math.toRadians(180)))
                                        .turn(Math.toRadians(180))
                                        .lineToLinearHeading(new Pose2d(-58, -35, Math.toRadians(0)))
                                        .lineToLinearHeading(new Pose2d(38, -35, Math.toRadians(0)))

                                        //                                        .splineToConstantHeading(new Vector2d(10, -34), Math.toRadians(-180))
//                                        .lineToLinearHeading(new Pose2d(-34, -60, Math.toRadians(0)))
                                        //                                            .lineToLinearHeading(new Pose2d(-0, -60, Math.toRadians(0)))
//                                        .lineToLinearHeading(new Pose2d(38, -58, Math.toRadians(0)))
                                        .build()
                        );

                mm.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                        .setDarkMode(true)
                        .setBackgroundAlpha(0.95f)
                        .addEntity(myBot)
                        .start();
            }
            else if (detection == 2) {
                RoadRunnerBotEntity myBot = new DefaultBotBuilder(mm)
                        // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                        .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                        .followTrajectorySequence(drive ->
                                        drive.trajectorySequenceBuilder(new Pose2d(-34, -60, Math.toRadians(90)))
                                                .lineToLinearHeading(new Pose2d(-34, -35, Math.toRadians(90)))
                                                .lineToLinearHeading(new Pose2d(38, -35, Math.toRadians(0)))
                                                .lineToLinearHeading(new Pose2d(-58, -35, Math.toRadians(0)))
                                                .lineToLinearHeading(new Pose2d(38, -35, Math.toRadians(0)))

                                                //                                        .splineToConstantHeading(new Vector2d(10, -34), Math.toRadians(-180))
//                                        .lineToLinearHeading(new Pose2d(-34, -60, Math.toRadians(0)))
                                                //                                            .lineToLinearHeading(new Pose2d(-0, -60, Math.toRadians(0)))
//                                        .lineToLinearHeading(new Pose2d(38, -58, Math.toRadians(0)))
                                                .build()
                        );

                mm.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                        .setDarkMode(true)
                        .setBackgroundAlpha(0.95f)
                        .addEntity(myBot)
                        .start();
            }
            else if (detection == 3) {
                RoadRunnerBotEntity myBot = new DefaultBotBuilder(mm)
                        // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                        .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                        .followTrajectorySequence(drive ->
                                        drive.trajectorySequenceBuilder(new Pose2d(-34, -60, Math.toRadians(90)))
                                                .lineToLinearHeading(new Pose2d(-34, -35, Math.toRadians(0)))
                                                .lineToLinearHeading(new Pose2d(38, -35, Math.toRadians(0)))
                                                .lineToLinearHeading(new Pose2d(-58, -35, Math.toRadians(0)))
                                                .lineToLinearHeading(new Pose2d(38, -35, Math.toRadians(0)))

                                                //                                        .splineToConstantHeading(new Vector2d(10, -34), Math.toRadians(-180))
//                                        .lineToLinearHeading(new Pose2d(-34, -60, Math.toRadians(0)))
                                                //                                            .lineToLinearHeading(new Pose2d(-0, -60, Math.toRadians(0)))
//                                        .lineToLinearHeading(new Pose2d(38, -58, Math.toRadians(0)))
                                                .build()
                        );

                mm.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                        .setDarkMode(true)
                        .setBackgroundAlpha(0.95f)
                        .addEntity(myBot)
                        .start();
            }
        }
        else if (auto == 2) {
            RoadRunnerBotEntity myBot = new DefaultBotBuilder(mm)
                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                    .followTrajectorySequence(drive ->
                                    drive.trajectorySequenceBuilder(new Pose2d(13, -60, Math.toRadians(-180)))
//                                        .splineToConstantHeading(new Vector2d(10, -34), Math.toRadians(-180))
                                            .lineToLinearHeading(new Pose2d(10, -34, Math.toRadians(-180)))
                                            .lineToLinearHeading(new Pose2d(38, -34, Math.toRadians(0)))

                                            .build()
                    );

            mm.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                    .setDarkMode(true)
                    .setBackgroundAlpha(0.95f)
                    .addEntity(myBot)
                    .start();
        }
        else if (auto == 3) {
            RoadRunnerBotEntity myBot = new DefaultBotBuilder(mm)
                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                    .followTrajectorySequence(drive ->
                                    drive.trajectorySequenceBuilder(new Pose2d(13, -60, Math.toRadians(-180)))
//                                        .splineToConstantHeading(new Vector2d(10, -34), Math.toRadians(-180))
                                            .lineToLinearHeading(new Pose2d(10, -34, Math.toRadians(-180)))
                                            .lineToLinearHeading(new Pose2d(38, -34, Math.toRadians(0)))

                                            .build()
                    );

            mm.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                    .setDarkMode(true)
                    .setBackgroundAlpha(0.95f)
                    .addEntity(myBot)
                    .start();
        }
        else if (auto == 4) {
            RoadRunnerBotEntity myBot = new DefaultBotBuilder(mm)
                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                    .followTrajectorySequence(drive ->
                                    drive.trajectorySequenceBuilder(new Pose2d(13, -60, Math.toRadians(-180)))
//                                        .splineToConstantHeading(new Vector2d(10, -34), Math.toRadians(-180))
                                            .lineToLinearHeading(new Pose2d(10, -34, Math.toRadians(-180)))
                                            .lineToLinearHeading(new Pose2d(38, -34, Math.toRadians(0)))

                                            .build()
                    );

            mm.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                    .setDarkMode(true)
                    .setBackgroundAlpha(0.95f)
                    .addEntity(myBot)
                    .start();
        }
    }
}