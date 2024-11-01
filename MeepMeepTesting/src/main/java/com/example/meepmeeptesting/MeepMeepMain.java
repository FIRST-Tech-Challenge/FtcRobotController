package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.ColorScheme;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepMain {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);

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
        RoadRunnerBotEntity botEntity = CreateBotEntity(meepMeep, "blue");
        botEntity.runAction(botEntity.getDrive().actionBuilder(new Pose2d(-15, 63, Math.toRadians(-90)))
                .splineTo(new Vector2d(-9,42), Math.toRadians(-90))

                //hook preloaded specimen
                .waitSeconds(1)

                .strafeTo(new Vector2d(-12,42))
                .splineToSplineHeading(new Pose2d(-36,24, Math.toRadians(0)), Math.toRadians(-90))
                .splineTo(new Vector2d(-42,12), Math.toRadians(180))
                .strafeTo(new Vector2d(-45,12))
                .strafeTo(new Vector2d(-45,48))
                .strafeTo(new Vector2d(-45,24))
                .splineTo(new Vector2d(-57,12), Math.toRadians(180))
                .strafeTo(new Vector2d(-57,48))
                .splineTo(new Vector2d(-63,63), Math.toRadians(180))
                .build());

        return botEntity;
    }

    //generates path for BlueLeft
    private static RoadRunnerBotEntity BlueLeft(MeepMeep meepMeep)
    {
        RoadRunnerBotEntity botEntity = CreateBotEntity(meepMeep, "blue");
        botEntity.runAction(botEntity.getDrive().actionBuilder(new Pose2d(0, 0, 0))

                .build());

        return botEntity;
    }

    //generates path for RedRight
    private static RoadRunnerBotEntity RedRight(MeepMeep meepMeep)
    {
        RoadRunnerBotEntity botEntity = CreateBotEntity(meepMeep, "red");
        botEntity.runAction(botEntity.getDrive().actionBuilder(new Pose2d(20, -65, 0))

                .build());

        return botEntity;
    }

    //generates path for RedLeft
    private static RoadRunnerBotEntity RedLeft(MeepMeep meepMeep)
    {
        RoadRunnerBotEntity botEntity = CreateBotEntity(meepMeep, "red");
        botEntity.runAction(botEntity.getDrive().actionBuilder(new Pose2d(0, 0, 0))

                .build());

        return botEntity;
    }
    private static RoadRunnerBotEntity CreateBotEntity(MeepMeep meepMeep, String color)
    {
        ColorScheme c;
        if (color.equals("red"))
        {
            c = new ColorSchemeRedLight();
        }
        else
        {
            c = new ColorSchemeBlueLight();
        }
        return new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(c)
                .build();

    }


}