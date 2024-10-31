package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTemplate {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(exampleRoute(meepMeep))
                .start();
    }

    private static RoadRunnerBotEntity exampleRoute(MeepMeep meepMeep)
    {
        RoadRunnerBotEntity botEntity = CreateBotEntity(meepMeep);
        botEntity.runAction(botEntity.getDrive().actionBuilder(new Pose2d(0, 0, 0))

                .build());

        return botEntity;
    }
    private static RoadRunnerBotEntity CreateBotEntity(MeepMeep meepMeep)
    {
        return new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();
    }


}
