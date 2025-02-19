package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepMecanumTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(95, 95, 3.2, 3.6, 16)
                .setDimensions(17.25,17.25)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0.000, -61.875, Math.PI / 2 ))

                // score FIRST SPECIMEN
                .lineToYLinearHeading(-31.125, Math.PI / 2)

                // drive to SAMPLES
                .setTangent(-Math.PI /2)
                .splineToConstantHeading(new Vector2d(35.25, -31.125), Math.PI/2)
                .waitSeconds(0)
                .splineToConstantHeading(new Vector2d(42.375, -8.625), -Math.PI/2)
                .waitSeconds(0)
                .lineToYConstantHeading(-47)
                .waitSeconds(0)
                .splineToConstantHeading(new Vector2d(52.375, -8.625), -Math.PI/2)
                .waitSeconds(0)
                .lineToYConstantHeading(-47)
                .waitSeconds(0)
                .splineToConstantHeading(new Vector2d(61.875, -8.625), -Math.PI/2)
                .waitSeconds(0)
                .lineToYConstantHeading(-49)
                .setTangent(5 * Math.PI / 6)

                // move and turn to intake specimen
                .splineToLinearHeading(new Pose2d(47, -54, -Math.PI / 4), Math.PI)

                // score SECOND SPECIMEN
                .setTangent(Math.PI)
                .splineToLinearHeading(new Pose2d(15, -31.125, Math.PI/2), Math.PI/2)

                // move to STAGING POSITION
                .setTangent(-Math.PI / 2)
                .splineToLinearHeading(new Pose2d(47, -54, -Math.PI / 4), 0)

                // score THIRD SPECIMEN
                .setTangent(Math.PI)
                .splineToLinearHeading(new Pose2d(13.5, -31.125, Math.PI/2), Math.PI/2)

                // move to STAGING POSITION
                .setTangent(-Math.PI / 2)
                .splineToLinearHeading(new Pose2d(47, -54, -Math.PI / 4), 0)

                // score FOURTH SPECIMEN
                .setTangent(Math.PI)
                .splineToLinearHeading(new Pose2d(10.5, -31.125, Math.PI/2), Math.PI/2)

                // move to STAGING POSITION
                .setTangent(-Math.PI / 2)
                .splineToLinearHeading(new Pose2d(47, -54, -Math.PI / 4), 0)

                // score FIFTH SPECIMEN
                .setTangent(Math.PI)
                .splineToLinearHeading(new Pose2d(9, -31.125, Math.PI/2), Math.PI/2)

                // move to STAGING POSITION
                .setTangent(-Math.PI / 2)
                .splineToLinearHeading(new Pose2d(47, -54, -Math.PI / 4), 0)

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}