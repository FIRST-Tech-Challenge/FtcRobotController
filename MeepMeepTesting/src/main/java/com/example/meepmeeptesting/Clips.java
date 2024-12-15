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
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-13.5, 60, Math.toRadians(270)))
                        //todo finish this
                        //Slides Up
                        .splineToConstantHeading(new Vector2d(-7,32),Math.toRadians(270))
                        .waitSeconds(5)
                        //slides down
                        //Open claw
                        .lineTo(new Vector2d(-7,34))
                        .setTangent(Math.toRadians(90))
                        .waitSeconds(.5)
                        //slides all the way down
                        .splineToConstantHeading(new Vector2d(-27,38),Math.toRadians(180))
                        .setTangent(Math.toRadians(180))
                        .splineToLinearHeading(new Pose2d(-46,13,Math.toRadians(90)),Math.toRadians(170))
                        //foward
                        .setTangent(Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(-46,59),Math.toRadians(90))
                        .setTangent(Math.toRadians(270))
                        .splineToConstantHeading(new Vector2d(-46,48),Math.toRadians(270))
                        .setTangent(Math.toRadians(90))
                        .waitSeconds(3)
                        .splineToConstantHeading(new Vector2d(-46,65),Math.toRadians(90))
                        .waitSeconds(5)
                        //slides up
                        //open claw
                        //close claw

                        //Slides Up
                        .setTangent(Math.toRadians(270))
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