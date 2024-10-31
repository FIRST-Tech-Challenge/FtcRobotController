package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepMain {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(BlueRight(meepMeep))
                .addEntity(BlueLeft(meepMeep))
                .addEntity(RedRight(meepMeep))
                .addEntity(RedLeft(meepMeep))
                .start();
    }

    //generates path for BlueRight
    private static RoadRunnerBotEntity BlueRight(MeepMeep meepMeep)
    {
        RoadRunnerBotEntity botEntity = CreateBotEntity(meepMeep);
        botEntity.runAction(botEntity.getDrive().actionBuilder(new Pose2d(-15, 63, Math.toRadians(-90)))
                .splineTo(new Vector2d(-15,55), Math.toRadians(-90))
                .build());

        return botEntity;
    }

    //generates path for BlueLeft
    private static RoadRunnerBotEntity BlueLeft(MeepMeep meepMeep)
    {
        RoadRunnerBotEntity botEntity = CreateBotEntity(meepMeep);
        botEntity.runAction(botEntity.getDrive().actionBuilder(new Pose2d(0, 0, 0))

                .build());

        return botEntity;
    }

    //generates path for RedRight
    private static RoadRunnerBotEntity RedRight(MeepMeep meepMeep)
    {
        RoadRunnerBotEntity botEntity = CreateBotEntity(meepMeep);
        botEntity.runAction(botEntity.getDrive().actionBuilder(new Pose2d(0, 0, 0))

                .build());

        return botEntity;
    }

    //generates path for RedLeft
    private static RoadRunnerBotEntity RedLeft(MeepMeep meepMeep)
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
