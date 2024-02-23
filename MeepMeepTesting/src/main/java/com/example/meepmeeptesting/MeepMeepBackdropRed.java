package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepBackdropRed {
    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(45, 45, Math.toRadians(60), Math.toRadians(60), 15.00)
                .setDimensions(15, 17)

                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(new Vector2d(12, -58.5), Math.toRadians(90)))

                                // Left Works
                            .lineToSplineHeading(new Pose2d(new Vector2d(12, -32.5), Math.toRadians(150)))
                            .turn(Math.toRadians(-60))
                           // .lineToSplineHeading(new Pose2d(new Vector2d(12, -34.5), Math.toRadians(45)))
                            //.lineToSplineHeading(new Pose2d(new Vector2d(12, -37), Math.toRadians(90)))

                            .back(10)

                            //.splineToLinearHeading(new Pose2d(new Vector2d(46, -36), Math.toRadians(180)), Math.toRadians(-60))

                            //.splineToLinearHeading(new Pose2d(new Vector2d(46, -30), Math.toRadians(180)), Math.toRadians(-60))

                            .splineToLinearHeading(new Pose2d(new Vector2d(46, -26), Math.toRadians(180)), Math.toRadians(60))

                            //.splineToLinearHeading(new Pose2d(new Vector2d(46, -34), Math.toRadians(180)), Math.toRadians(-60))

                            .splineToLinearHeading(new Pose2d(new Vector2d(56, -56), Math.toRadians(180)), Math.toRadians(0))
                            .build()
                );



        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}