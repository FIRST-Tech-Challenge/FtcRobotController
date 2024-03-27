package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepBackdropBlue {
    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(45, 45, Math.toRadians(60), Math.toRadians(60), 15.00)
                .setDimensions(15, 17)

                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(new Vector2d(12, 58.5), Math.toRadians(270)))
                                        .setTangent(Math.toRadians(-40))
                                        .splineToLinearHeading(new Pose2d(35, 25, Math.toRadians(180)), Math.toRadians(-70))
                                        .back(5)
                                        .setTangent(Math.toRadians(0))
                                        .splineToLinearHeading(new Pose2d(48, 40, Math.toRadians(180)), Math.toRadians(0))
                                        .setTangent(Math.toRadians(120))
                                        .splineToConstantHeading(new Vector2d(30, 58), Math.toRadians(180))
                                      .splineToSplineHeading(new Pose2d(-34, 58, Math.toRadians(180)),Math.toRadians(180))
                                        .splineToConstantHeading(new Vector2d(-56, 36), Math.toRadians(180))
//                                        .setReversed(true)

                                        .splineToConstantHeading(new Vector2d(-24, 58),Math.toRadians(0))
                                        .splineToSplineHeading(new Pose2d(30, 58, Math.toRadians(180)),Math.toRadians(0))
                                        .splineToConstantHeading(new Vector2d(48, 56),Math.toRadians(0))
//
                                        .splineToConstantHeading(new Vector2d(30, 58), Math.toRadians(180))
                                        .splineToSplineHeading(new Pose2d(-34, 58, Math.toRadians(180)),Math.toRadians(180))
                                        .splineToConstantHeading(new Vector2d(-56, 36), Math.toRadians(180))

                                        .splineToConstantHeading(new Vector2d(-24, 58),Math.toRadians(0))
                                        .splineToSplineHeading(new Pose2d(30, 58, Math.toRadians(180)),Math.toRadians(0))
                                        .splineToConstantHeading(new Vector2d(48, 56),Math.toRadians(0))

                                        .build()
                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}