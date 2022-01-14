package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepRedWarehouse {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(45, 60, Math.toRadians(60), Math.toRadians(60), 16.4)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(new Vector2d(13.5, -63),Math.toRadians(90)))
                                .strafeTo(new Vector2d(-15, -44))
                                .lineToSplineHeading(new Pose2d(new Vector2d(5, -60), Math.toRadians(180)))
                                .splineToLinearHeading(new Pose2d( new Vector2d(48, -66), Math.toRadians(180)), Math.toRadians(0))
                                .lineTo(new Vector2d(60, -66))
                                .lineTo(new Vector2d(15, -66))
                                .splineToSplineHeading(new Pose2d(-11, -48, Math.toRadians(90)), Math.toRadians(90))
                                .lineToSplineHeading(new Pose2d(new Vector2d(5, -60), Math.toRadians(180)))
                                .splineToLinearHeading(new Pose2d( new Vector2d(48, -66), Math.toRadians(180)), Math.toRadians(0))
                                .lineTo(new Vector2d(60, -66))
                                .lineTo(new Vector2d(15, -66))
                                .splineToSplineHeading(new Pose2d(-11, -48, Math.toRadians(90)), Math.toRadians(90))
                                .lineToSplineHeading(new Pose2d(new Vector2d(5, -60), Math.toRadians(180)))
                                .splineToLinearHeading(new Pose2d( new Vector2d(48, -66), Math.toRadians(180)), Math.toRadians(0))
                                .build()
                )
                ;

        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}