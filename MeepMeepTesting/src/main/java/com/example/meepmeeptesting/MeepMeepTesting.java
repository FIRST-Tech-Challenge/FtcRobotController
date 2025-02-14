package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 18)
                .setDimensions(18, 18)
                .build();
        RoadRunnerBotEntity myBot2 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180),18)
                .setDimensions(18, 18)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(6+(18.0/2.0) -30, -(72-(18.0/2.0)), Math.toRadians(90)))
                .splineToLinearHeading(new Pose2d(-7, -31.5, Math.toRadians(90)), Math.toRadians(90))
                //intake
                .strafeTo(new Vector2d(-7, -40))
                .splineToLinearHeading(new Pose2d(-27,-58,Math.toRadians(125)), Math.toRadians(180))
                //score
                .splineToLinearHeading(new Pose2d(-32, -58, Math.toRadians(180)), Math.toRadians(180))
                .build());
        myBot2.runAction(myBot.getDrive().actionBuilder(new Pose2d(6+(18.0/2.0), -(72-(18.0/2.0)), Math.toRadians(90)))

                .build());
        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
//                .addEntity(myBot2)
                .start();
    }
}