package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class Bucket {
    public static void main(String[] args) {
        System.setProperty("sun.java2d.opengl", "true");
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new ActualBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(37, 60, Math.toRadians(90)))
                        .setTangent(Math.toRadians(270))
                        .lineTo(new Vector2d(37,53))
                        .splineToConstantHeading(new Vector2d(47,40),Math.toRadians(280))           
                        .waitSeconds(3)
                         //pickup
                        .setTangent(Math.toRadians(90))
                        .splineToLinearHeading(new Pose2d(53,58,Math.toRadians(45)),Math.toRadians(70))
                        .waitSeconds(1)
                        //Slides up
                        //open claw
                        .setTangent(Math.toRadians(225))
                        .splineToConstantHeading(new Vector2d(49,53),Math.toRadians(90))
                        //slides down
                        .splineToLinearHeading(new Pose2d(58,40,Math.toRadians(90)),Math.toRadians(270))
                        .waitSeconds(3)
                        //pickup
                        .setTangent(Math.toRadians(90))
                        .splineToLinearHeading(new Pose2d(53,58,Math.toRadians(45)),Math.toRadians(70))
                        .waitSeconds(1)
                        //Slides up
                        //open claw
                        .setTangent(Math.toRadians(225))
                        .splineToConstantHeading(new Vector2d(49,53),Math.toRadians(90))

                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}