package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-12, -60, 1.5708))
                .lineToY(-36)
                .turn(Math.toRadians(-90))
                .lineToX(0)
                .turn(Math.toRadians(90))
                //score specimen
                .strafeTo(new Vector2d( -50, -36))
                //grab sample
                .turn(Math.toRadians(135))
                .strafeTo(new Vector2d( -60, -60))
                //score first sample
                .strafeTo(new Vector2d( -55, -36))
                .turn(Math.toRadians(-135))
                //grab sample
                .turn(Math.toRadians(150))
                .strafeTo(new Vector2d( -60, -60))
                //score second sample
                .strafeTo(new Vector2d(-50, -24))
                .turn(Math.toRadians(-60))
                .lineToX(-60)
                //grab sample
                .lineToX(-50)
                .turn(Math.toRadians(60))
                .strafeTo(new Vector2d( -60, -60))
                //score third sample
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}