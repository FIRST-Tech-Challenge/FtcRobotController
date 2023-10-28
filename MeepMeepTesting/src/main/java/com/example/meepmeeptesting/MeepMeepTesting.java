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

        Vector2d parkingPosBlue = new Vector2d(56,56);
        Vector2d parkingPosRed = new Vector2d(56,-56);
        Vector2d scoreBlue = new Vector2d(42,38);
        Vector2d scoreRed = new Vector2d(42,-34);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPoseBlueFar)
                                .splineTo(new Vector2d(-34,38), Math.toRadians(0))
                                .turn(Math.toRadians(-90))
                                //.UNSTABLE_addTemporalMarkerOffset(-0.3, this::dropPurplePixel)
                                .waitSeconds(1.5)
                                .turn(Math.toRadians(90))
                                //.UNSTABLE_addTemporalMarkerOffset(0,this::stopNoodles)
                                .splineTo(scoreBlue,Math.toRadians(0))
                                //.UNSTABLE_addTemporalMarkerOffset(-0.1,this::stageScore)
                                .waitSeconds(1.5)
                                //.UNSTABLE_addTemporalMarkerOffset(0,this::stopNoodles)
                                .splineTo(parkingPosBlue,Math.toRadians(0))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(1f)
                .addEntity(myBot)
                .start();
    }
}