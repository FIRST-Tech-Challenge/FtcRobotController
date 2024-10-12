package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepPilesBlue {
    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(45, 30, Math.toRadians(60), Math.toRadians(60), 15.00)
                .setDimensions(15, 17)

                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(new Vector2d(-35, 58.5), Math.toRadians(270)))
                                        .lineToSplineHeading(new Pose2d(-38, 50, Math.toRadians(270-15)))
                                        .lineToSplineHeading(new Pose2d(-35,30, Math.toRadians(270)))
                                        .lineToSplineHeading(new Pose2d(-35, 10, Math.toRadians(180)))
//                                        .lineToLinearHeading(new Pose2d(50, 10, Math.toRadians(180)))
//                                        .strafeTo(new Vector2d(50, 24))
//                                        //deposit here
//                                        .splineToConstantHeading(new Vector2d(40, 10),Math.toRadians(180))
//                                        .splineToConstantHeading(new Vector2d(-40, 10),Math.toRadians(180))
//                                        .lineTo(new Vector2d(50, 10))
//                                        .strafeTo(new Vector2d(50, 24))
                                        //.lineToSplineHeading(new Pose2d(-38, 17, Math.toRadians(-60)))
//                                        .lineToSplineHeading(new Pose2d(-32, 17, Math.toRadians(-120)))
//
//                                        .lineToSplineHeading(new Pose2d(-35, 12, Math.toRadians(180)))
//                                        .back(65)
//                                        .splineToLinearHeading(new Pose2d(48, 36, Math.toRadians(180)), Math.toRadians(0))
                                        .build()
                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}