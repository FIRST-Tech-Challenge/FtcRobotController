package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d startPoseBlueFar = new Pose2d(-52, 52, 0);
        Pose2d startPoseBlueClose = new Pose2d(38, 56, 0);
        Pose2d startPoseRedClose = new Pose2d(10, -52, 0);
        Pose2d startPoseRedFar = new Pose2d(-52, -48, 0);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPoseRedClose)
                                .splineTo(new Vector2d(10,-34),Math.toRadians(0))
                                //.addTemporalMarker(this::dropPurplePixel)
                                .splineTo(new Vector2d(42,-34),Math.toRadians(0))
                                //.addTemporalMarker(this::stageScore)
                                //.addTemporalMarker(this::stopNoodles)
                                .splineTo(new Vector2d(56,-56),Math.toRadians(0))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(1f)
                .addEntity(myBot)
                .start();
    }
}