package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(45, 60, Math.toRadians(60), Math.toRadians(60), 16.4)


                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(new Vector2d(37.5, -64.25), Math.toRadians(90)))
                                .splineToLinearHeading(new Pose2d( new Vector2d(14,-60), Math.toRadians(90)), Math.toRadians(180))
                                .lineToLinearHeading(new Pose2d(new Vector2d(10.75,-35.5),Math.toRadians(135)))
                                .turn(Math.toRadians(-45))
                                .splineToLinearHeading(new Pose2d( new Vector2d(20,-14), Math.toRadians(0)),Math.toRadians(0))
                                .lineToLinearHeading(new Pose2d(new Vector2d(59,-14),Math.toRadians(0)))
                                .splineToLinearHeading(new Pose2d(new Vector2d(31,-11),Math.toRadians(315)),Math.toRadians(155))

                                .build()
                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}