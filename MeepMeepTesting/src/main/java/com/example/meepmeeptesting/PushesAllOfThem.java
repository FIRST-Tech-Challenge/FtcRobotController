package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class PushesAllOfThem {
    public static void main(String[] args) {
        System.setProperty("sun.java2d.opengl", "true");
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new ActualBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-15.5, 60, Math.toRadians(270)))
                        .splineToConstantHeading(new Vector2d(-6.9,31),Math.toRadians(270))
                        .setTangent(90)
                       //Slides down open claw

                        .splineToConstantHeading(new Vector2d(-32,33),Math.toRadians(270))
                        .waitSeconds(1)

                        .setTangent(Math.toRadians(325))
                        .splineToLinearHeading(new Pose2d(-46,13,Math.toRadians(90)),Math.toRadians(170))

                        // 1st pixel
                        .strafeTo(new Vector2d(-46,59))
                        .strafeTo(new Vector2d(-46,13))

                        //2nd Pixel
                        .strafeLeft(10)
                        .strafeTo(new Vector2d(-56,59))
                        .setTangent(Math.toRadians(270))
                        .strafeTo(new Vector2d(-56,13))


                        //3rd Pixel
                        .strafeLeft(10)
                        .strafeTo(new Vector2d(-66,59))
                        .setTangent(Math.toRadians(270))
                        .strafeTo(new Vector2d(-66,13))


                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
