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
                .setConstraints(45, 45, Math.toRadians(60), Math.toRadians(60), 12.5)
                .setDimensions(15, 17.8)

                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(new Vector2d(16, 61.2), Math.toRadians(270)))
                                        .setTangent(Math.toRadians(-40))
                                        .splineToLinearHeading(new Pose2d(10, 30, Math.toRadians(180)), Math.toRadians(-70))
                                        .waitSeconds(0.5)

                                        .setTangent(Math.toRadians(0))
                                        .splineToLinearHeading(new Pose2d(48, 29, Math.toRadians(180)), Math.toRadians(0))
                                        .waitSeconds(0.5)

//                                        //2+2
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
//                                        //2+4
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
//                                        //Park
//
//                                        .setTangent(Math.toRadians(180))
//                                        .splineToLinearHeading(new Pose2d(44, 60, Math.toRadians(180)), Math.toRadians(0))

                                        .build()
                );

        RoadRunnerBotEntity BlueBackdropTrussC = new DefaultBotBuilder(meepMeep)
                .setConstraints(45, 45, Math.toRadians(60), Math.toRadians(60), 12.5)
                .setDimensions(15, 17.8)

                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(new Vector2d(16, 61.2), Math.toRadians(270)))
                                        .setTangent(Math.toRadians(-90))
                                        .splineToLinearHeading(new Pose2d(16, 34, Math.toRadians(-90)), Math.toRadians(-90))
                                        .waitSeconds(0.5)

                                        .setTangent(Math.toRadians(90))
                                        .splineToLinearHeading(new Pose2d(50, 34, Math.toRadians(180)), Math.toRadians(0))
                                        .waitSeconds(0.5)

//                                //2+2
//
//                                .setTangent(Math.toRadians(180))
//                                .splineToLinearHeading(new Pose2d(10, 58, Math.toRadians(180)), Math.toRadians(180))
//                                .splineToLinearHeading(new Pose2d(-30, 58, Math.toRadians(180)), Math.toRadians(180))
//                                .splineToLinearHeading(new Pose2d(-58, 35.2, Math.toRadians(180)), Math.toRadians(-140))
//                                .waitSeconds(0.5)
//
//                                .setTangent(Math.toRadians(40))
//                                .splineToLinearHeading(new Pose2d(-30, 58, Math.toRadians(180)), Math.toRadians(0))
//                                .splineToLinearHeading(new Pose2d(20, 58, Math.toRadians(180)), Math.toRadians(0))
//
//                                .setTangent(Math.toRadians(0))
//                                .splineToLinearHeading(new Pose2d(48, 38, Math.toRadians(180)), Math.toRadians(0))
//
//                                //2+4
//
//                                .setTangent(Math.toRadians(180))
//                                .splineToLinearHeading(new Pose2d(10, 58, Math.toRadians(180)), Math.toRadians(180))
//                                .splineToLinearHeading(new Pose2d(-30, 58, Math.toRadians(180)), Math.toRadians(180))
//                                .splineToLinearHeading(new Pose2d(-58, 35.2, Math.toRadians(180)), Math.toRadians(-140))
//                                .waitSeconds(0.5)
//
//                                .setTangent(Math.toRadians(40))
//                                .splineToLinearHeading(new Pose2d(-30, 58, Math.toRadians(180)), Math.toRadians(0))
//                                .splineToLinearHeading(new Pose2d(20, 58, Math.toRadians(180)), Math.toRadians(0))
//
//                                .setTangent(Math.toRadians(0))
//                                .splineToLinearHeading(new Pose2d(48, 38, Math.toRadians(180)), Math.toRadians(0))
//
//                                //Park
//
//                                .setTangent(Math.toRadians(180))
//                                .splineToLinearHeading(new Pose2d(44, 60, Math.toRadians(180)), Math.toRadians(0))

                                        .build()
                );

        RoadRunnerBotEntity BlueBackdropTrussL = new DefaultBotBuilder(meepMeep)
                .setConstraints(45, 45, Math.toRadians(60), Math.toRadians(60), 12.5)
                .setDimensions(15, 17.8)

                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(new Vector2d(16, 61.2), Math.toRadians(270)))
                                        .setTangent(Math.toRadians(-90))
                                        .splineToLinearHeading(new Pose2d(33, 35, Math.toRadians(-180)), Math.toRadians(-90))
                                        .waitSeconds(0.5)

                                        .setTangent(Math.toRadians(0))
                                        .splineToLinearHeading(new Pose2d(48, 40, Math.toRadians(180)), Math.toRadians(0))
                                        .waitSeconds(0.5)

//                                //2+2
//
                                .setTangent(Math.toRadians(180))
                                .splineToLinearHeading(new Pose2d(10, 58, Math.toRadians(180)), Math.toRadians(180))
                                .splineToLinearHeading(new Pose2d(-30, 58, Math.toRadians(180)), Math.toRadians(180))
                                .splineToLinearHeading(new Pose2d(-58, 35.2, Math.toRadians(180)), Math.toRadians(-140))
                                .waitSeconds(0.5)
//
                                        .setTangent(Math.toRadians(90))
                                        .splineToLinearHeading(new Pose2d(-30, 62, Math.toRadians(180)), Math.toRadians(0))
                                        .splineToLinearHeading(new Pose2d(20, 62, Math.toRadians(180)), Math.toRadians(0))
                                        .setTangent(Math.toRadians(0))
                                        .splineToLinearHeading(new Pose2d(48, 38, Math.toRadians(180)), Math.toRadians(0))

//
//                                .setTangent(Math.toRadians(0))
//                                .splineToLinearHeading(new Pose2d(48, 38, Math.toRadians(180)), Math.toRadians(0))
//
//                                //2+4
//
//                                .setTangent(Math.toRadians(180))
//                                .splineToLinearHeading(new Pose2d(10, 58, Math.toRadians(180)), Math.toRadians(180))
//                                .splineToLinearHeading(new Pose2d(-30, 58, Math.toRadians(180)), Math.toRadians(180))
//                                .splineToLinearHeading(new Pose2d(-58, 35.2, Math.toRadians(180)), Math.toRadians(-140))
//                                .waitSeconds(0.5)
//
//                                .setTangent(Math.toRadians(40))
//                                .splineToLinearHeading(new Pose2d(-30, 58, Math.toRadians(180)), Math.toRadians(0))
//                                .splineToLinearHeading(new Pose2d(20, 58, Math.toRadians(180)), Math.toRadians(0))
//
//                                .setTangent(Math.toRadians(0))
//                                .splineToLinearHeading(new Pose2d(48, 38, Math.toRadians(180)), Math.toRadians(0))
//
//                                //Park
//
//                                .setTangent(Math.toRadians(180))
//                                .splineToLinearHeading(new Pose2d(44, 60, Math.toRadians(180)), Math.toRadians(0))

                                        .build()
                );

        RoadRunnerBotEntity BlueBackdropStageR = new DefaultBotBuilder(meepMeep)
                .setConstraints(45, 45, Math.toRadians(60), Math.toRadians(60), 12.5)
                .setDimensions(15, 17.8)

                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(new Vector2d(16, 61.2), Math.toRadians(270)))
                                        .setTangent(Math.toRadians(-40))
                                        .splineToLinearHeading(new Pose2d(10, 30, Math.toRadians(180)), Math.toRadians(-70))
                                        .waitSeconds(0.5)

                                        .setTangent(Math.toRadians(0))
                                        .splineToLinearHeading(new Pose2d(48, 29, Math.toRadians(180)), Math.toRadians(0))
                                        .waitSeconds(0.5)

//                                //2+2
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
//                                //2+4
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
//
////                                //2+5
////
////                                .setTangent(Math.toRadians(180))
////                                .splineToLinearHeading(new Pose2d(10, 11.5, Math.toRadians(180)), Math.toRadians(180))
////                                .splineToLinearHeading(new Pose2d(-57, 11.5, Math.toRadians(180)), Math.toRadians(180))
////                                .waitSeconds(0.5)
////
////                                .setTangent(Math.toRadians(0))
////                                .splineToLinearHeading(new Pose2d(10, 11.5, Math.toRadians(180)), Math.toRadians(0))
////                                .splineToLinearHeading(new Pose2d(47, 28, Math.toRadians(180)), Math.toRadians(0))
////                                .waitSeconds(0.5)
//
//                                //2+6
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

        RoadRunnerBotEntity BlueBackdropStageC = new DefaultBotBuilder(meepMeep)
                .setConstraints(45, 45, Math.toRadians(60), Math.toRadians(60), 12.5)
                .setDimensions(15, 17.8)

                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(new Vector2d(16, 61.2), Math.toRadians(270)))
                                        .setTangent(Math.toRadians(-90))
                                        .splineToLinearHeading(new Pose2d(16, 34, Math.toRadians(-90)), Math.toRadians(-90))
                                        .waitSeconds(0.5)

                                        .setTangent(Math.toRadians(90))
                                        .splineToLinearHeading(new Pose2d(48, 34, Math.toRadians(180)), Math.toRadians(0))
                                        .waitSeconds(0.5)

                                        //2+2

                                        .setTangent(Math.toRadians(180))
                                        .splineToLinearHeading(new Pose2d(20, 11.5, Math.toRadians(180)), Math.toRadians(180))
                                        .splineToLinearHeading(new Pose2d(-57, 11.5, Math.toRadians(180)), Math.toRadians(180))
                                        .waitSeconds(0.5)

                                        .setTangent(Math.toRadians(0))
                                        .splineToLinearHeading(new Pose2d(20, 11.5, Math.toRadians(180)), Math.toRadians(0))
                                        .splineToLinearHeading(new Pose2d(47, 28, Math.toRadians(180)), Math.toRadians(0))
                                        .waitSeconds(0.5)

                                        //2+4

                                        .setTangent(Math.toRadians(180))
                                        .splineToLinearHeading(new Pose2d(20, 11.5, Math.toRadians(180)), Math.toRadians(180))
                                        .splineToLinearHeading(new Pose2d(-57, 11.5, Math.toRadians(180)), Math.toRadians(180))
                                        .waitSeconds(0.5)

                                        .setTangent(Math.toRadians(0))
                                        .splineToLinearHeading(new Pose2d(20, 11.5, Math.toRadians(180)), Math.toRadians(0))
                                        .splineToLinearHeading(new Pose2d(47, 28, Math.toRadians(180)), Math.toRadians(0))
                                        .waitSeconds(0.5)


//                                //2+5
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

                                        //2+6

                                        .setTangent(Math.toRadians(180))
                                        .splineToLinearHeading(new Pose2d(20, 11.5, Math.toRadians(180)), Math.toRadians(180))
                                        .splineToLinearHeading(new Pose2d(-40, 11.5, Math.toRadians(180)), Math.toRadians(180))
                                        .splineToLinearHeading(new Pose2d(-58, 23, Math.toRadians(180)), Math.toRadians(140))
                                        .waitSeconds(0.5)

                                        .setTangent(Math.toRadians(-40))
                                        .splineToLinearHeading(new Pose2d(-45, 11.5, Math.toRadians(180)), Math.toRadians(0))
                                        .splineToLinearHeading(new Pose2d(20, 11.5, Math.toRadians(180)), Math.toRadians(0))
                                        .splineToLinearHeading(new Pose2d(47, 28, Math.toRadians(180)), Math.toRadians(0))
                                        .waitSeconds(0.5)


                                        .build()
                );

        RoadRunnerBotEntity BlueBackdropStageL = new DefaultBotBuilder(meepMeep)
                .setConstraints(45, 45, Math.toRadians(200), Math.toRadians(180), 12.5)
                .setDimensions(15, 17.8)

                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(new Vector2d(16, 61.2), Math.toRadians(270)))
                                        .setTangent(Math.toRadians(-90))
                                        .splineToLinearHeading(new Pose2d(33, 32, Math.toRadians(-180)), Math.toRadians(-90))
                                        .waitSeconds(0.5)

                                        .setTangent(Math.toRadians(0))
                                        .splineToLinearHeading(new Pose2d(48, 40, Math.toRadians(180)), Math.toRadians(0))
                                        .waitSeconds(0.5)

                                        //2+2

                                        .setTangent(Math.toRadians(180))
                                        .splineToLinearHeading(new Pose2d(30, 11.5, Math.toRadians(180)), Math.toRadians(180))
                                        .splineToLinearHeading(new Pose2d(-57, 11.5, Math.toRadians(180)), Math.toRadians(180))
                                        .waitSeconds(0.5)

                                        .setTangent(Math.toRadians(0))
                                        .splineToLinearHeading(new Pose2d(20, 11.5, Math.toRadians(180)), Math.toRadians(0))
                                        .splineToLinearHeading(new Pose2d(47, 28, Math.toRadians(180)), Math.toRadians(0))
                                        .waitSeconds(0.5)

                                        //2+4

                                        .setTangent(Math.toRadians(180))
                                        .splineToLinearHeading(new Pose2d(20, 11.5, Math.toRadians(180)), Math.toRadians(180))
                                        .splineToLinearHeading(new Pose2d(-57, 11.5, Math.toRadians(180)), Math.toRadians(180))
                                        .waitSeconds(0.5)

                                        .setTangent(Math.toRadians(0))
                                        .splineToLinearHeading(new Pose2d(20, 11.5, Math.toRadians(180)), Math.toRadians(0))
                                        .splineToLinearHeading(new Pose2d(47, 28, Math.toRadians(180)), Math.toRadians(0))
                                        .waitSeconds(0.5)


//                                //2+5
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

                                        //2+6

                                        .setTangent(Math.toRadians(180))
                                        .splineToLinearHeading(new Pose2d(20, 11.5, Math.toRadians(180)), Math.toRadians(180))
                                        .splineToLinearHeading(new Pose2d(-40, 11.5, Math.toRadians(180)), Math.toRadians(180))
                                        .splineToLinearHeading(new Pose2d(-58, 23, Math.toRadians(180)), Math.toRadians(140))
                                        .waitSeconds(0.5)

                                        .setTangent(Math.toRadians(-40))
                                        .splineToLinearHeading(new Pose2d(-45, 11.5, Math.toRadians(180)), Math.toRadians(0))
                                        .splineToLinearHeading(new Pose2d(20, 11.5, Math.toRadians(180)), Math.toRadians(0))
                                        .splineToLinearHeading(new Pose2d(47, 28, Math.toRadians(180)), Math.toRadians(0))
                                        .waitSeconds(0.5)


                                        .build()
                );

        RoadRunnerBotEntity BlueFarStageR = new DefaultBotBuilder(meepMeep)
                .setConstraints(45, 45, Math.toRadians(60), Math.toRadians(60), 12.50)
                .setDimensions(15, 17.8)

                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(new Vector2d(-39, 61.2), Math.toRadians(270)))
                                .setTangent(Math.toRadians(-50))
                                .splineToLinearHeading(new Pose2d(-47, 15, Math.toRadians(-90)), Math.toRadians(-110))
                                .waitSeconds(0.5)

                                .setTangent(Math.toRadians(-90))
                                .splineToLinearHeading(new Pose2d(-57, 11.5, Math.toRadians(180)), Math.toRadians(180))
                                .waitSeconds(0.5)

                                .setTangent(Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(10, 11.5, Math.toRadians(180)), Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(47, 28, Math.toRadians(180)), Math.toRadians(0))
                                .waitSeconds(0.5)

                                //2+3

//
                                .setTangent(Math.toRadians(180))
                                .splineToLinearHeading(new Pose2d(42, 12, Math.toRadians(180)), Math.toRadians(0))




                                .build()
                );

        RoadRunnerBotEntity BlueFarStageC = new DefaultBotBuilder(meepMeep)
                .setConstraints(45, 45, Math.toRadians(60), Math.toRadians(60), 12.50)
                .setDimensions(15, 17.8)

                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(new Vector2d(-39, 61.2), Math.toRadians(270)))
                                .setTangent(Math.toRadians(-50))
                                .splineToLinearHeading(new Pose2d(-45, 14, Math.toRadians(-120)), Math.toRadians(-110))
                                .waitSeconds(0.5)

                                .setTangent(Math.toRadians(-140))
                                .splineToLinearHeading(new Pose2d(-57, 11.5, Math.toRadians(180)), Math.toRadians(180))
                                .waitSeconds(0.5)

                                .setTangent(Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(20, 11.5, Math.toRadians(180)), Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(47, 34, Math.toRadians(180)), Math.toRadians(0))
                                .waitSeconds(0.5)

                                //2+3

//                                .setTangent(Math.toRadians(180))
//                                .splineToLinearHeading(new Pose2d(20, 11.5, Math.toRadians(180)), Math.toRadians(180))
//                                .splineToLinearHeading(new Pose2d(-57, 11.5, Math.toRadians(180)), Math.toRadians(180))
//                                .waitSeconds(0.5)
//
//                                .setTangent(Math.toRadians(0))
//                                .splineToLinearHeading(new Pose2d(20, 11.5, Math.toRadians(180)), Math.toRadians(0))
//                                .splineToLinearHeading(new Pose2d(47, 28, Math.toRadians(180)), Math.toRadians(0))
//                                .waitSeconds(0.5)
//
//                                //2+5
//
//                                .setTangent(Math.toRadians(180))
//                                .splineToLinearHeading(new Pose2d(20, 11.5, Math.toRadians(180)), Math.toRadians(180))
//                                .splineToLinearHeading(new Pose2d(-57, 11.5, Math.toRadians(180)), Math.toRadians(180))
//                                .waitSeconds(0.5)
//
//                                .setTangent(Math.toRadians(0))
//                                .splineToLinearHeading(new Pose2d(20, 11.5, Math.toRadians(180)), Math.toRadians(0))
//                                .splineToLinearHeading(new Pose2d(47, 28, Math.toRadians(180)), Math.toRadians(0))
//                                .waitSeconds(0.5)

                                .setTangent(Math.toRadians(180))
                                .splineToLinearHeading(new Pose2d(44, 15, Math.toRadians(180)), Math.toRadians(0))


                                .build()
                );

        RoadRunnerBotEntity BlueFarStageL = new DefaultBotBuilder(meepMeep)
                .setConstraints(45, 45, Math.toRadians(60), Math.toRadians(60), 12.50)
                .setDimensions(15, 17.8)

                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(new Vector2d(-39, 61.2), Math.toRadians(270)))
                                .setTangent(Math.toRadians(-110))
                                .splineToLinearHeading(new Pose2d(-35, 35, Math.toRadians(-180)), Math.toRadians(-30))
                                .waitSeconds(0.5)

                                .setTangent(Math.toRadians(-90))
                                .splineToLinearHeading(new Pose2d(-57, 11.5, Math.toRadians(180)), Math.toRadians(180))
                                .waitSeconds(0.5)

                                .setTangent(Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(30, 11.5, Math.toRadians(180)), Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(47, 38, Math.toRadians(180)), Math.toRadians(0))
                                .waitSeconds(0.5)

                                //2+3

                                .setTangent(Math.toRadians(180))
                                .splineToLinearHeading(new Pose2d(30, 11.5, Math.toRadians(180)), Math.toRadians(180))
                                .splineToLinearHeading(new Pose2d(-57, 11.5, Math.toRadians(180)), Math.toRadians(180))
                                .waitSeconds(0.5)

                                .setTangent(Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(20, 11.5, Math.toRadians(180)), Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(47, 28, Math.toRadians(180)), Math.toRadians(0))
                                .waitSeconds(0.5)

                                //2+5

                                .setTangent(Math.toRadians(180))
                                .splineToLinearHeading(new Pose2d(20, 11.5, Math.toRadians(180)), Math.toRadians(180))
                                .splineToLinearHeading(new Pose2d(-57, 11.5, Math.toRadians(180)), Math.toRadians(180))
                                .waitSeconds(0.5)

                                .setTangent(Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(20, 11.5, Math.toRadians(180)), Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(47, 28, Math.toRadians(180)), Math.toRadians(0))
                                .waitSeconds(0.5)




                                .build()
                );



        RoadRunnerBotEntity BlueFarTrussR = new DefaultBotBuilder(meepMeep)
                .setConstraints(45, 45, Math.toRadians(60), Math.toRadians(60), 12.5)
                .setDimensions(15, 17.8)

                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(new Vector2d(-39, 61.2), Math.toRadians(270)))
                                .setTangent(Math.toRadians(-90))
                                .splineToLinearHeading(new Pose2d(-47, 38, Math.toRadians(-90)), Math.toRadians(-90))
                                .waitSeconds(0.5)
                                //2+0

                                .setTangent(Math.toRadians(70))
                                .splineToLinearHeading(new Pose2d(-30, 58, Math.toRadians(180)), Math.toRadians(0))
                                .setTangent(Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(20, 58, Math.toRadians(180)), Math.toRadians(0))

                                .setTangent(Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(48, 38, Math.toRadians(180)), Math.toRadians(0))
                                .waitSeconds(0.5)

//                                //2+2
//
//                                .setTangent(Math.toRadians(180))
//                                .splineToLinearHeading(new Pose2d(10, 58, Math.toRadians(180)), Math.toRadians(180))
//                                .splineToLinearHeading(new Pose2d(-30, 58, Math.toRadians(180)), Math.toRadians(180))
//                                .splineToLinearHeading(new Pose2d(-58, 35.2, Math.toRadians(180)), Math.toRadians(-140))
//                                .waitSeconds(0.5)
//
//                                .setTangent(Math.toRadians(40))
//                                .splineToLinearHeading(new Pose2d(-30, 58, Math.toRadians(180)), Math.toRadians(0))
//                                .splineToLinearHeading(new Pose2d(20, 58, Math.toRadians(180)), Math.toRadians(0))
//
//                                .setTangent(Math.toRadians(0))
//                                .splineToLinearHeading(new Pose2d(48, 38, Math.toRadians(180)), Math.toRadians(0))
//                                .waitSeconds(0.5)
//
//                                //2+4
//
//                                .setTangent(Math.toRadians(180))
//                                .splineToLinearHeading(new Pose2d(10, 58, Math.toRadians(180)), Math.toRadians(180))
//                                .splineToLinearHeading(new Pose2d(-30, 58, Math.toRadians(180)), Math.toRadians(180))
//                                .splineToLinearHeading(new Pose2d(-58, 35.2, Math.toRadians(180)), Math.toRadians(-140))
//                                .waitSeconds(0.5)

                                .setTangent(Math.toRadians(40))
//                                .splineToLinearHeading(new Pose2d(-30, 58, Math.toRadians(180)), Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(50, 58, Math.toRadians(180)), Math.toRadians(0))
                                .waitSeconds(0.5)

                                .build()
                );

        RoadRunnerBotEntity BlueFarTrussL = new DefaultBotBuilder(meepMeep)
                .setConstraints(45, 45, Math.toRadians(60), Math.toRadians(60), 12.5)
                .setDimensions(15, 17.8)

                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(new Vector2d(-39, 61.2), Math.toRadians(270)))
                                        .setTangent(Math.toRadians(270))
                                        .splineToLinearHeading(new Pose2d(-31, 38, Math.toRadians(310)), Math.toRadians(-90))
                                        .waitSeconds(0.5)
                                        //2+0

                                        .setTangent(Math.toRadians(70))
                                        .splineToLinearHeading(new Pose2d(-30, 58, Math.toRadians(180)), Math.toRadians(0))
                                        .setTangent(Math.toRadians(0))
                                        .splineToLinearHeading(new Pose2d(20, 58, Math.toRadians(180)), Math.toRadians(0))

                                        .setTangent(Math.toRadians(0))
                                        .splineToLinearHeading(new Pose2d(50, 42, Math.toRadians(180)), Math.toRadians(0))
                                        .waitSeconds(0.5)

//                                //2+2
//
//                                .setTangent(Math.toRadians(180))
//                                .splineToLinearHeading(new Pose2d(10, 58, Math.toRadians(180)), Math.toRadians(180))
//                                .splineToLinearHeading(new Pose2d(-30, 58, Math.toRadians(180)), Math.toRadians(180))
//                                .splineToLinearHeading(new Pose2d(-58, 35.2, Math.toRadians(180)), Math.toRadians(-140))
//                                .waitSeconds(0.5)
//
//                                .setTangent(Math.toRadians(40))
//                                .splineToLinearHeading(new Pose2d(-30, 58, Math.toRadians(180)), Math.toRadians(0))
//                                .splineToLinearHeading(new Pose2d(20, 58, Math.toRadians(180)), Math.toRadians(0))
//
//                                .setTangent(Math.toRadians(0))
//                                .splineToLinearHeading(new Pose2d(48, 38, Math.toRadians(180)), Math.toRadians(0))
//                                .waitSeconds(0.5)
//
//                                //2+4
//
//                                .setTangent(Math.toRadians(180))
//                                .splineToLinearHeading(new Pose2d(10, 58, Math.toRadians(180)), Math.toRadians(180))
//                                .splineToLinearHeading(new Pose2d(-30, 58, Math.toRadians(180)), Math.toRadians(180))
//                                .splineToLinearHeading(new Pose2d(-58, 35.2, Math.toRadians(180)), Math.toRadians(-140))
//                                .waitSeconds(0.5)

                                        .setTangent(Math.toRadians(40))
//                                .splineToLinearHeading(new Pose2d(-30, 58, Math.toRadians(180)), Math.toRadians(0))
                                        .splineToLinearHeading(new Pose2d(50, 58, Math.toRadians(180)), Math.toRadians(0))
                                        .waitSeconds(0.5)

                                        .build()
                );

        RoadRunnerBotEntity BlueFarTrussM = new DefaultBotBuilder(meepMeep)
                .setConstraints(45, 45, Math.toRadians(60), Math.toRadians(60), 12.5)
                .setDimensions(15, 17.8)

                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(new Vector2d(-39, 61.2), Math.toRadians(270)))
                                        .setTangent(Math.toRadians(-90))
                                        .splineToLinearHeading(new Pose2d(-34, 34, Math.toRadians(-90)), Math.toRadians(-70))
                                        .waitSeconds(0.5)
                                        //2+0

                                        .setTangent(Math.toRadians(150))
                                        .splineToLinearHeading(new Pose2d(-30, 58, Math.toRadians(180)), Math.toRadians(20))
                                        .setTangent(Math.toRadians(0))
                                        .splineToLinearHeading(new Pose2d(20, 58, Math.toRadians(180)), Math.toRadians(0))

                                        .setTangent(Math.toRadians(0))
                                        .splineToLinearHeading(new Pose2d(50, 42, Math.toRadians(180)), Math.toRadians(0))
                                        .waitSeconds(0.5)

//                                //2+2
//
//                                .setTangent(Math.toRadians(180))
//                                .splineToLinearHeading(new Pose2d(10, 58, Math.toRadians(180)), Math.toRadians(180))
//                                .splineToLinearHeading(new Pose2d(-30, 58, Math.toRadians(180)), Math.toRadians(180))
//                                .splineToLinearHeading(new Pose2d(-58, 35.2, Math.toRadians(180)), Math.toRadians(-140))
//                                .waitSeconds(0.5)
//
//                                .setTangent(Math.toRadians(40))
//                                .splineToLinearHeading(new Pose2d(-30, 58, Math.toRadians(180)), Math.toRadians(0))
//                                .splineToLinearHeading(new Pose2d(20, 58, Math.toRadians(180)), Math.toRadians(0))
//
//                                .setTangent(Math.toRadians(0))
//                                .splineToLinearHeading(new Pose2d(48, 38, Math.toRadians(180)), Math.toRadians(0))
//                                .waitSeconds(0.5)
//
//                                //2+4
//
//                                .setTangent(Math.toRadians(180))
//                                .splineToLinearHeading(new Pose2d(10, 58, Math.toRadians(180)), Math.toRadians(180))
//                                .splineToLinearHeading(new Pose2d(-30, 58, Math.toRadians(180)), Math.toRadians(180))
//                                .splineToLinearHeading(new Pose2d(-58, 35.2, Math.toRadians(180)), Math.toRadians(-140))
//                                .waitSeconds(0.5)

                                        .setTangent(Math.toRadians(40))
//                                .splineToLinearHeading(new Pose2d(-30, 58, Math.toRadians(180)), Math.toRadians(0))
                                        .splineToLinearHeading(new Pose2d(50, 58, Math.toRadians(180)), Math.toRadians(0))
                                        .waitSeconds(0.5)

                                        .build()
                );



                        RoadRunnerBotEntity RedBackdropTrussL = new DefaultBotBuilder(meepMeep)
                                .setConstraints(45, 45, Math.toRadians(60), Math.toRadians(60), 12.5)
                                .setDimensions(15, 17.8)

                                .followTrajectorySequence(drive ->
                                                drive.trajectorySequenceBuilder(new Pose2d(new Vector2d(16, -61.2), Math.toRadians(90)))
                                                        .setTangent(Math.toRadians(-40-180))
                                                        .splineToLinearHeading(new Pose2d(10, -30, Math.toRadians(180)), Math.toRadians(-70-180))
                                                        .waitSeconds(0.5)

                                                        .setTangent(Math.toRadians(0))
                                                        .splineToLinearHeading(new Pose2d(50, -29, Math.toRadians(180)), Math.toRadians(0))
                                                        .waitSeconds(0.5)

//                                        //2+2
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
//                                        //2+4
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
//                                        //Park
//
//                                        .setTangent(Math.toRadians(180))
//                                        .splineToLinearHeading(new Pose2d(44, 60, Math.toRadians(180)), Math.toRadians(0))

                                                        .build()
                                );

        RoadRunnerBotEntity RedBackdropTrussR = new DefaultBotBuilder(meepMeep)
                .setConstraints(45, 45, Math.toRadians(60), Math.toRadians(60), 12.5)
                .setDimensions(15, 17.8)

                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(new Vector2d(16, -61.2), Math.toRadians(90)))
                                        .setTangent(Math.toRadians(-40-180))
                                        .splineToLinearHeading(new Pose2d(35, -29, Math.toRadians(180)), Math.toRadians(-70-180))
                                        .waitSeconds(0.5)

                                        .setTangent(Math.toRadians(0))
                                        .splineToLinearHeading(new Pose2d(50, -42, Math.toRadians(180)), Math.toRadians(0))
                                        .waitSeconds(0.5)

//                                        //2+2
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
//                                        //2+4
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
//                                        //Park
//
//                                        .setTangent(Math.toRadians(180))
//                                        .splineToLinearHeading(new Pose2d(44, 60, Math.toRadians(180)), Math.toRadians(0))

                                        .build()
                );

        RoadRunnerBotEntity RedBackdropTrussC = new DefaultBotBuilder(meepMeep)
                .setConstraints(45, 45, Math.toRadians(60), Math.toRadians(60), 12.5)
                .setDimensions(15, 17.8)

                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(new Vector2d(16, -61.2), Math.toRadians(90)))
                                        .setTangent(Math.toRadians((90)))
                                        .splineToLinearHeading(new Pose2d(16, -34, Math.toRadians(90)), Math.toRadians(90))
                                        .waitSeconds(0.5)

                                        .setTangent(Math.toRadians(-90))
                                        .splineToLinearHeading(new Pose2d(50, -32, Math.toRadians(180)), Math.toRadians(0))
                                        .waitSeconds(0.5)

//                                        //2+2
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
//                                        //2+4
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
//                                        //Park
//
//                                        .setTangent(Math.toRadians(180))
//                                        .splineToLinearHeading(new Pose2d(44, 60, Math.toRadians(180)), Math.toRadians(0))

                                        .build()
                );

        RoadRunnerBotEntity RedBackdropTrussCOpposite = new DefaultBotBuilder(meepMeep)
                .setConstraints(45, 45, Math.toRadians(60), Math.toRadians(60), 12.5)
                .setDimensions(15, 17.8)

                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(new Vector2d(16, -61.2), Math.toRadians(90)))
                                        .setTangent(Math.toRadians((90)))
                                        .splineToLinearHeading(new Pose2d(16, -34, Math.toRadians(90)), Math.toRadians(90))
                                        .waitSeconds(0.5)

                                        .setTangent(Math.toRadians(-90))
                                        .splineToLinearHeading(new Pose2d(50, -32, Math.toRadians(180)), Math.toRadians(0))
                                        .waitSeconds(0.5)

//                                        //2+2
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
//                                        //2+4
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
//                                        //Park
//
                                        .setTangent(Math.toRadians(180))
                                        .splineToLinearHeading(new Pose2d(50, -10, Math.toRadians(180)), Math.toRadians(0))

                                        .build()
                );


        RoadRunnerBotEntity RedFarSideGateC = new DefaultBotBuilder(meepMeep)
                .setConstraints(45, 45, Math.toRadians(60), Math.toRadians(60), 12.5)
                .setDimensions(15, 17.8)

                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(new Vector2d(-39, -61.2), Math.toRadians(90)))
                                        .setTangent(Math.toRadians(90))
                                .splineToLinearHeading(new Pose2d(-45, -15, Math.toRadians(130)), Math.toRadians(90))
                                        .waitSeconds(0.5)

                                        .setTangent(Math.toRadians(90))
                                        .splineToLinearHeading(new Pose2d(-57, -11.5, Math.toRadians(180)), Math.toRadians(180))

                                        .setTangent(Math.toRadians(0))
                                        .splineToLinearHeading(new Pose2d(20, -11.5, Math.toRadians(180)), Math.toRadians(0))
                                        .splineToLinearHeading(new Pose2d(47, -34, Math.toRadians(180)), Math.toRadians(0))

                                        .setTangent(Math.toRadians(180))
                                        .splineToLinearHeading(new Pose2d(44, -15, Math.toRadians(180)), Math.toRadians(0))

//                                        //2+2
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
//                                        //2+4
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
//                                        //Park
//
//                                        .setTangent(Math.toRadians(180))
//                                        .splineToLinearHeading(new Pose2d(44, 60, Math.toRadians(180)), Math.toRadians(0))

                                        .build()
                );

        RoadRunnerBotEntity RedBFarSideGateL = new DefaultBotBuilder(meepMeep)
                .setConstraints(45, 45, Math.toRadians(60), Math.toRadians(60), 12.5)
                .setDimensions(15, 17.8)

                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(new Vector2d(-39, -61.2), Math.toRadians(90)))
                                        .setTangent(Math.toRadians(70))
                                        .splineToLinearHeading(new Pose2d(-47, -15, Math.toRadians(90)), Math.toRadians(130))
                                        .waitSeconds(0.5)

                                        .setTangent(Math.toRadians(90))
                                        .splineToLinearHeading(new Pose2d(-57, -11.5, Math.toRadians(180)), Math.toRadians(180))

                                        .setTangent(Math.toRadians(0))
                                        .splineToLinearHeading(new Pose2d(25, -11.5, Math.toRadians(180)), Math.toRadians(0))
                                        .splineToLinearHeading(new Pose2d(40, -29, Math.toRadians(180)), Math.toRadians(0))

                                        .setTangent(Math.toRadians(180))
                                        .splineToLinearHeading(new Pose2d(44, -15, Math.toRadians(180)), Math.toRadians(0))

//                                        //2+2
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
//                                        //2+4
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
//                                        //Park
//
//                                        .setTangent(Math.toRadians(180))
//                                        .splineToLinearHeading(new Pose2d(44, 60, Math.toRadians(180)), Math.toRadians(0))

                                        .build()
                );

        RoadRunnerBotEntity RedBFarSideGateR = new DefaultBotBuilder(meepMeep)
                .setConstraints(45, 45, Math.toRadians(60), Math.toRadians(60), 12.5)
                .setDimensions(15, 17.8)

                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(new Vector2d(-39, -61.2), Math.toRadians(90)))
                                       .setTangent(Math.toRadians(150))
                                        .splineToLinearHeading(new Pose2d(-35, -35, Math.toRadians(-180)), Math.toRadians(30))
                                        .waitSeconds(0.5)

                                        .setTangent(Math.toRadians(90))
                                        .splineToLinearHeading(new Pose2d(-57, -11.5, Math.toRadians(180)), Math.toRadians(180))
                                        .waitSeconds(0.5)

                                        .setTangent(Math.toRadians(0))
                                        .splineToLinearHeading(new Pose2d(30, -11.5, Math.toRadians(180)), Math.toRadians(0))
                                        .splineToLinearHeading(new Pose2d(47, -38, Math.toRadians(180)), Math.toRadians(0))
                                        .waitSeconds(0.5)

                                        //2+3

                                        .setTangent(Math.toRadians(180))
                                        .splineToLinearHeading(new Pose2d(30, -11.5, Math.toRadians(180)), Math.toRadians(180))
                                        .splineToLinearHeading(new Pose2d(-57, -11.5, Math.toRadians(180)), Math.toRadians(180))
                                        .waitSeconds(0.5)

                                        .setTangent(Math.toRadians(0))
                                        .splineToLinearHeading(new Pose2d(30, -11.5, Math.toRadians(180)), Math.toRadians(0))
                                        .splineToLinearHeading(new Pose2d(47, -28, Math.toRadians(180)), Math.toRadians(0))
                                        .waitSeconds(0.5)

                                        //2+5

                                        .setTangent(Math.toRadians(180))
                                        .splineToLinearHeading(new Pose2d(30, -11.5, Math.toRadians(180)), Math.toRadians(180))
                                        .splineToLinearHeading(new Pose2d(-57, -11.5, Math.toRadians(180)), Math.toRadians(180))
                                        .waitSeconds(0.5)

                                        .setTangent(Math.toRadians(0))
                                        .splineToLinearHeading(new Pose2d(30, -11.5, Math.toRadians(180)), Math.toRadians(0))
                                        .splineToLinearHeading(new Pose2d(47, -28, Math.toRadians(180)), Math.toRadians(0))
                                        .waitSeconds(0.5)




                                        .build()
                );

        RoadRunnerBotEntity RedBackdropOuttakeL= new DefaultBotBuilder(meepMeep)
                .setConstraints(45, 45, Math.toRadians(60), Math.toRadians(60), 12.5)
                .setDimensions(15, 17.8)

                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(new Vector2d(16, -61.2), Math.toRadians(90)))
                                .setTangent(Math.toRadians(120))
                                .splineToLinearHeading(new Pose2d(13, -35, Math.toRadians(-180)), Math.toRadians(30))
                                .waitSeconds(0.5)


                                .setTangent(Math.toRadians(0))

                                .splineToLinearHeading(new Pose2d(47, -38, Math.toRadians(180)), Math.toRadians(0))
                                .waitSeconds(0.5)






                                .build()
                );

        RoadRunnerBotEntity RedBackdropOuttakeM= new DefaultBotBuilder(meepMeep)
                .setConstraints(45, 45, Math.toRadians(60), Math.toRadians(60), 12.5)
                .setDimensions(15, 17.8)

                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(new Vector2d(16, -61.2), Math.toRadians(90)))
                                .setTangent(Math.toRadians(90))
                                .splineToLinearHeading(new Pose2d(21, -15, Math.toRadians(0)), Math.toRadians(0))
                                .waitSeconds(0.5)


                                .setTangent(Math.toRadians(0))

                                .splineToLinearHeading(new Pose2d(47, -38, Math.toRadians(180)), Math.toRadians(0))
                                .waitSeconds(0.5)






                                .build()
                );


        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(BlueFarStageR )
                .start();
    }
}