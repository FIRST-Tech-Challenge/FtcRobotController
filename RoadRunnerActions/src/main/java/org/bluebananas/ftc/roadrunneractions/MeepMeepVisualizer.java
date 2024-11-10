package org.bluebananas.ftc.roadrunneractions;

import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.ColorScheme;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepVisualizer {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(BlueRightOption1(meepMeep))
                .addEntity(BlueLeft(meepMeep))
                .addEntity(RedRightOption2(meepMeep))
                .addEntity(RedLeft(meepMeep))
                .start();
    }

    //begin auto runs

    //generates path for BlueRightOption1
    private static RoadRunnerBotEntity BlueRightOption1(MeepMeep meepMeep)
    {
        RoadRunnerBotEntity botEntity = CreateBotEntity(meepMeep, "blue");
        botEntity.runAction(ActionBuilder.BlueRightOption1(botEntity.getDrive()::actionBuilder));
        return botEntity;
    }

    //generates path for BlueLeft
    private static RoadRunnerBotEntity BlueLeft(MeepMeep meepMeep)
    {
        RoadRunnerBotEntity botEntity = CreateBotEntity(meepMeep, "blue");
        botEntity.runAction(ActionBuilder.BlueBasket(botEntity.getDrive()::actionBuilder));
        return botEntity;
    }

    //generates path for RedRightOption2
    private static RoadRunnerBotEntity RedRightOption2(MeepMeep meepMeep) {
        RoadRunnerBotEntity botEntity = CreateBotEntity(meepMeep, "red");
        botEntity.runAction(ActionBuilder.RedRightOption2(botEntity.getDrive()::actionBuilder));
        return botEntity;
    }

    //generates path for RedLeft
    private static RoadRunnerBotEntity RedLeft(MeepMeep meepMeep) {
        RoadRunnerBotEntity botEntity = CreateBotEntity(meepMeep, "red");
        botEntity.runAction(ActionBuilder.RedLeft(botEntity.getDrive()::actionBuilder));
        return botEntity;
    }

    //end auto runs


    //generates botEntities
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