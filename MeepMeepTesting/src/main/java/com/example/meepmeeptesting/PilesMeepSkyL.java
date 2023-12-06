package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class PilesMeepSkyL {
    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(45, 45, Math.toRadians(60), Math.toRadians(60), 15.00)
                .setDimensions(15, 17)

                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(new Vector2d(-35, 58.5), Math.toRadians(270)))

                                        // Left Works
                                        .turn(Math.toRadians(-13))
                                        .turn(Math.toRadians(13))
                                        .forward(40)
                                        .splineToLinearHeading(new Pose2d(-16, 11, Math.toRadians(180)), Math.toRadians(0))
                                        .back(55)
                                        .strafeRight(30)
                                        .strafeLeft(30)
                                        .forward(55)
                                        .build()
                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}