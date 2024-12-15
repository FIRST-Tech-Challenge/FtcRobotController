package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class Park {
    public static void main(String[] args) {
        System.setProperty("sun.java2d.opengl", "true");
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new ActualBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(7, -60, Math.toRadians(270)))
                        //todo finish this
                        .setTangent(Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(31,-23),Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(47,-6),0)
                        .setTangent(Math.toRadians(270))
                        .splineToConstantHeading(new Vector2d(47,-52),Math.toRadians(270))
                        .setTangent(Math.toRadians(90))

                        .splineToConstantHeading(new Vector2d(53,-6),Math.toRadians(45))
                        .setTangent(Math.toRadians(270))
                        .splineToConstantHeading(new Vector2d(53,-52),Math.toRadians(270))

                        .setTangent(Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(64,-6),Math.toRadians(45))
                        .setTangent(Math.toRadians(270))
                        .splineToConstantHeading(new Vector2d(64d,-52),Math.toRadians(270))


                        //.lineTo(new Vector2d(60,-60))

                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}