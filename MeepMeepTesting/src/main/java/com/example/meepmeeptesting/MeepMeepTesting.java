package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();
        Pose2d startColor = new Pose2d(12,-60,Math.toRadians(90));
        myBot.runAction(myBot.getDrive().actionBuilder(startColor)
                //COLOR SIDE ROUTE
            //Hang Preload
                .lineToY(-34)
                .waitSeconds(3) //Hang Specimin

            //Pick up new sample
                .strafeTo(new Vector2d(48.5,-38))
                //Claw/Arm stuff
                .waitSeconds(3)//Pick up Spike mark

            //Drop of sample for HP
                .lineToYSplineHeading(-52,Math.toRadians(-90))
                .waitSeconds(1)//Drop Sample
                .lineToY(-44)
                .waitSeconds(5)//Place sample in HP zone, wait for HP

            //Pick up specimin from HP
                .lineToY(-60)
                //Extend/prepare claw here
                .strafeTo(new Vector2d(60,-60))
                //Close claw
                .waitSeconds(1)

            //Drive and hang specimin
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(12,-34,Math.toRadians(90)),0)//Travel to hanging bar
                .waitSeconds(3)// Hang Specimin Again

            //Park in asscent zone
                .strafeTo(new Vector2d(36,-36))
                .strafeTo(new Vector2d(36,-12))
                .strafeTo(new Vector2d(24,-12))

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}