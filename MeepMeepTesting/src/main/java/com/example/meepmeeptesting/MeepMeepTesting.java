package com.example.meepmeeptesting;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.ColorScheme;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Color;

public class MeepMeepTesting {
    public static void main(String[] args){
        MeepMeep MM = new MeepMeep(800);
        //creating bot
        RoadRunnerBotEntity bot = new DefaultBotBuilder(MM)
                .setConstraints(60,60,Math.toRadians(180),Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeRedDark())

                //creating trajectory sequence
                .followTrajectorySequence(drive ->
                    drive.trajectorySequenceBuilder(new Pose2d(0,0,0))
                            .forward(15)
                            .build()
                    );



        MM.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
          .setDarkMode(true)

          // Background opacity from 0-1
          .setBackgroundAlpha(1f)
          .addEntity(bot)
          .start();
    }




}