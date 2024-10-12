package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class BoardMeepSkyC {
    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(45, 45, Math.toRadians(60), Math.toRadians(60), 15.00)
                .setDimensions(15, 17)

                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(new Vector2d(16, 61.2), Math.toRadians(270)))
                                .setTangent(Math.toRadians(-40))
                                .splineToLinearHeading(new Pose2d(10, 30, Math.toRadians(180)), Math.toRadians(-70))
                                .waitSeconds(0.5)

                                .setTangent(Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(48, 29, Math.toRadians(180)), Math.toRadians(0))
                                .waitSeconds(0.5)
                                .build()
                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}