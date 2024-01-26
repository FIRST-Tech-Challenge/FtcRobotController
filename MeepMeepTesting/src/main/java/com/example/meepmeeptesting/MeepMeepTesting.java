package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, -60, Math.PI*1.5))
                                .lineToLinearHeading(new Pose2d(12, -30, Math.toRadians(180)))
                                .back(24)
                                .lineToLinearHeading(new Pose2d(31, -10, Math.toRadians(0)))
                                .lineToConstantHeading(new Vector2d(-36, -11))
                                .forward(82)
                                .strafeRight(40)
                                .build()

                        /*
                        drive.trajectorySequenceBuilder(new Pose2d(12, -60, Math.PI*1.5))
                               .lineToSplineHeading(new Pose2d(36, -24, Math.toRadians(180)))
                               .splineToConstantHeading(new Vector2d(11, -12), Math.toRadians(0))
                               .lineTo(new Vector2d(-36, -12))
                               .lineToSplineHeading(new Pose2d(-47, -12, Math.toRadians(0)))
                               .forward(90)
                               .strafeRight(48)
                               .forward(10)
                               .build()
                        */
                );



        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
