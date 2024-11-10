package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeep1 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(80, 70, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, -61.5, Math.toRadians(90)))
                        .lineToY(-34)
                        .waitSeconds(2)
                        .lineToY(-31)
                        .waitSeconds(1)
                        .lineToY(-34)
                        .strafeTo(new Vector2d(35,-35))
                        .strafeTo(new Vector2d(35,-10))
                        .splineTo(new Vector2d(47,-10),Math.toRadians(270))
                        .strafeTo(new Vector2d(47,-51))
                        .lineToY(-58)
                        .lineToY(-45)
                        .lineToY(-60.5)
                        .waitSeconds(2)
                        .lineToY(-45)
                        .splineTo(new Vector2d(2,-31), Math.toRadians(90))
                        .waitSeconds(1)
                        .lineToY(-40)
                        .strafeTo(new Vector2d(47,-61))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}