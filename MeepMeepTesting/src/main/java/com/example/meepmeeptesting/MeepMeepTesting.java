package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        System.setProperty("sun.java2d.opengl", "true");
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new ActualBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-10, 60, 0))
                        //todo finish this
                        .strafeLeft(-5)
                        //.turn(Math.toRadians(-90))
                        //.forward(30)
                        .splineToLinearHeading(new Pose2d(-35, 22, 0), -90)
                        .splineToLinearHeading(new Pose2d(-27, 5.8, 0), 0)
                        //.splineToLinearHeading(new Pose2d(-37, 12, 0), 90)
                        //.forward(-5)
                        .splineToLinearHeading(new Pose2d(-50, 5.8, Math.toRadians(15)), 0)
                        .splineToLinearHeading(new Pose2d(-57, 62,  Math.toRadians(-90)), 90)
                        //.splineToLinearHeading(new Pose2d(-57, 64,  Math.toRadians(-90)), 90)
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}