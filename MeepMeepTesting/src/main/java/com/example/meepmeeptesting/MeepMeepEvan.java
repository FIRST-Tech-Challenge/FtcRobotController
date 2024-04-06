package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepEvan {
    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity BlueBackdropTrussR = new DefaultBotBuilder(meepMeep)
                .setConstraints(50, 50, Math.toRadians(60), Math.toRadians(60), 15.00)
                .setDimensions(15, 17.8)

                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(new Vector2d(16, 61.2), Math.toRadians(270)))
                                        .setTangent(Math.toRadians(-40))
                                        .splineToLinearHeading(new Pose2d(10, 30, Math.toRadians(180)), Math.toRadians(-70))
                                        .waitSeconds(0.5)

                                        .setTangent(Math.toRadians(0))
                                        .splineToLinearHeading(new Pose2d(48, 29, Math.toRadians(180)), Math.toRadians(0))
                                        .waitSeconds(0.5)

//                                        .setTangent(Math.toRadians(180))
//                                        .splineToLinearHeading(new Pose2d(10, 58, Math.toRadians(180)), Math.toRadians(180))
//                                        .splineToLinearHeading(new Pose2d(-30, 58, Math.toRadians(180)), Math.toRadians(180))
//                                        .splineToLinearHeading(new Pose2d(-58, 35.2, Math.toRadians(180)), Math.toRadians(-140))
//                                        .waitSeconds(0.5)
//
//                                        .setTangent(Math.toRadians(40))
//                                        .splineToLinearHeading(new Pose2d(-30, 58, Math.toRadians(180)), Math.toRadians(0))
//                                        .splineToLinearHeading(new Pose2d(20, 58, Math.toRadians(180)), Math.toRadians(0))
//
//                                        .setTangent(Math.toRadians(0))
//                                        .splineToLinearHeading(new Pose2d(48, 38, Math.toRadians(180)), Math.toRadians(0))
//
//                                        .setTangent(Math.toRadians(180))
//                                        .splineToLinearHeading(new Pose2d(10, 58, Math.toRadians(180)), Math.toRadians(180))
//                                        .splineToLinearHeading(new Pose2d(-30, 58, Math.toRadians(180)), Math.toRadians(180))
//                                        .splineToLinearHeading(new Pose2d(-58, 35.2, Math.toRadians(180)), Math.toRadians(-140))
//                                        .waitSeconds(0.5)
//
//                                        .setTangent(Math.toRadians(40))
//                                        .splineToLinearHeading(new Pose2d(-30, 58, Math.toRadians(180)), Math.toRadians(0))
//                                        .splineToLinearHeading(new Pose2d(20, 58, Math.toRadians(180)), Math.toRadians(0))
//
//                                        .setTangent(Math.toRadians(0))
//                                        .splineToLinearHeading(new Pose2d(48, 38, Math.toRadians(180)), Math.toRadians(0))
//
//                                        .setTangent(Math.toRadians(180))
//                                        .splineToLinearHeading(new Pose2d(44, 60, Math.toRadians(180)), Math.toRadians(0))

                                        .build()
                );

        RoadRunnerBotEntity BlueBackdropStageR = new DefaultBotBuilder(meepMeep)
                .setConstraints(50, 50, Math.toRadians(60), Math.toRadians(60), 15.00)
                .setDimensions(15, 17.8)

                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(new Vector2d(16, 61.2), Math.toRadians(270)))
                                .setTangent(Math.toRadians(-40))
                                .splineToLinearHeading(new Pose2d(10, 30, Math.toRadians(180)), Math.toRadians(-70))
                                .waitSeconds(0.5)

//                                .setTangent(Math.toRadians(0))
//                                .splineToLinearHeading(new Pose2d(48, 29, Math.toRadians(180)), Math.toRadians(0))
//                                .waitSeconds(0.5)
//
//                                .setTangent(Math.toRadians(180))
//                                .splineToLinearHeading(new Pose2d(10, 11.5, Math.toRadians(180)), Math.toRadians(180))
//                                .splineToLinearHeading(new Pose2d(-57, 11.5, Math.toRadians(180)), Math.toRadians(180))
//                                .waitSeconds(0.5)
//
//                                .setTangent(Math.toRadians(0))
//                                .splineToLinearHeading(new Pose2d(10, 11.5, Math.toRadians(180)), Math.toRadians(0))
//                                .splineToLinearHeading(new Pose2d(47, 28, Math.toRadians(180)), Math.toRadians(0))
//                                .waitSeconds(0.5)
//
//                                .setTangent(Math.toRadians(180))
//                                .splineToLinearHeading(new Pose2d(10, 11.5, Math.toRadians(180)), Math.toRadians(180))
//                                .splineToLinearHeading(new Pose2d(-57, 11.5, Math.toRadians(180)), Math.toRadians(180))
//                                .waitSeconds(0.5)
//
//                                .setTangent(Math.toRadians(0))
//                                .splineToLinearHeading(new Pose2d(10, 11.5, Math.toRadians(180)), Math.toRadians(0))
//                                .splineToLinearHeading(new Pose2d(47, 28, Math.toRadians(180)), Math.toRadians(0))
//                                .waitSeconds(0.5)
//
//                                .setTangent(Math.toRadians(180))
//                                .splineToLinearHeading(new Pose2d(10, 11.5, Math.toRadians(180)), Math.toRadians(180))
//                                .splineToLinearHeading(new Pose2d(-40, 11.5, Math.toRadians(180)), Math.toRadians(180))
//                                .splineToLinearHeading(new Pose2d(-58, 23, Math.toRadians(180)), Math.toRadians(140))
//                                .waitSeconds(0.5)
//
//                                .setTangent(Math.toRadians(-40))
//                                .splineToLinearHeading(new Pose2d(-45, 11.5, Math.toRadians(180)), Math.toRadians(0))
//                                .splineToLinearHeading(new Pose2d(10, 11.5, Math.toRadians(180)), Math.toRadians(0))
//                                .splineToLinearHeading(new Pose2d(47, 28, Math.toRadians(180)), Math.toRadians(0))
//                                .waitSeconds(0.5)


                                .build()
                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(BlueBackdropTrussR)
                .start();
    }
}