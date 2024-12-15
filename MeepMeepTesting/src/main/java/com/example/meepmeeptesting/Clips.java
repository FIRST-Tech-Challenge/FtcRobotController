package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class Clips {
    public static void main(String[] args) {
        System.setProperty("sun.java2d.opengl", "true");
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new ActualBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-37, 60, Math.toRadians(270)))
                        //todo finish this
                        //Slides Up
                        .forward(1)
                        .splineToConstantHeading(new Vector2d(-7,32),Math.toRadians(270))
                        .waitSeconds(5)
                        //slides down
                        //Open claw
                        .back(3)
                        //slides all the way down
                        .splineToConstantHeading(new Vector2d(-27,38),Math.toRadians(180))
                        .splineToLinearHeading(new Pose2d(-46,13,Math.toRadians(90)),Math.toRadians(180))
                        .forward(45)
//                        .back(45)
//                        .strafeLeft(10)
//                        .forward(45)
//                        .back(45)
//                        .strafeLeft(7)
//                        .forward(45)
//                        .back(5)
//                        .strafeRight(15)
                        //slides up
                        //open claw
                        .back(10)
                        .waitSeconds(2)
                        .forward(15)
                        .waitSeconds(5)
                        //close claw
                        .setTangent(Math.toRadians(270))
                        //Slides Up
                        .splineToLinearHeading(new Pose2d(-7,32,Math.toRadians(270)),Math.toRadians(270))
                        //slides down
                        //Open claw

                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}