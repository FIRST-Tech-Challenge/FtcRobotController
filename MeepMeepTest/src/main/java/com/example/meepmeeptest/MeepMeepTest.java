package com.example.meepmeeptest;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTest {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(48,-58, 3.14159268979/2))
                        .lineTo(new Vector2d(48,-36))
                        .lineTo(new Vector2d(11,-33))
                        .lineTo(new Vector2d(58,-36))
                        .lineTo(new Vector2d(9,-33))
                        .lineTo(new Vector2d(69,-36))
                        .lineTo(new Vector2d(7,-33))
                        .waitSeconds(3)
                        .lineTo(new Vector2d(5,-33))
                        .waitSeconds(3)
                        .lineTo(new Vector2d(3,-33))
                        .waitSeconds(3)
                        .lineTo(new Vector2d(1,-33))
                        .waitSeconds(5)
                        .lineTo(new Vector2d(58,-58))
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}