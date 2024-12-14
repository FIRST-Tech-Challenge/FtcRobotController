package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

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
                        //get thing
                        .strafeLeft(-5)
                        .splineToLinearHeading(new Pose2d(-44, 22, 0), -90)
                        .splineToLinearHeading(new Pose2d(-28, 5.8, 0), 90)
                        //get hook
                        .splineToConstantHeading(new Vector2d(-52, 57), Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(-56, 57, Math.toRadians(-90)), 0)
                        //put hook
                        .splineToLinearHeading(new Pose2d(-55, 47, Math.toRadians(90)), 90)
                        .strafeRight(5)
                        .splineToConstantHeading(new Vector2d(0, 35), Math.toRadians(0))
                        //go back
                        .strafeLeft(40)
                        .splineToConstantHeading(new Vector2d(-28, 5.8), Math.toRadians(0))
                        .turn(Math.toRadians(-90))



                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}