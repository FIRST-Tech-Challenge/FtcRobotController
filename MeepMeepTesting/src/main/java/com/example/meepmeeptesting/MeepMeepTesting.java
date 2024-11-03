package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        //object and field background and screen size
        MeepMeep mm = new MeepMeep(600);

        //Declare a bot (you can declare how many you want)
        RoadRunnerBotEntity firstbot = new DefaultBotBuilder(mm)
                //giving it some color
                .setColorScheme(new ColorSchemeBlueDark())
                // setting the bot constrains (maxVel, maxAccel, maxAngVel track width)
                .setConstraints(60,60,Math.toRadians(180),Math.toRadians(180), 15).build();
        //setting the starting point in the row below. and the rest of the route after the starting point
        firstbot.runAction(firstbot.getDrive().actionBuilder(new Pose2d(23.7,69,Math.toRadians(270)))
                .splineToLinearHeading(new Pose2d(63.7,69,Math.toRadians(45)),0)
                //second gamePiece
                .splineToLinearHeading(new Pose2d(58.5,24.3,Math.toRadians(270)),0)
                .splineToLinearHeading(new Pose2d(63.7,69,Math.toRadians(45)),0)
                //third gamePiece
                .splineToLinearHeading(new Pose2d(48 , 24.3, Math.toRadians(270)),0)
                .splineToLinearHeading(new Pose2d(63.7,69,Math.toRadians(45)),0)
                .build());

        RoadRunnerBotEntity secondbot = new DefaultBotBuilder(mm)
                //giving it some color
                .setColorScheme(new ColorSchemeBlueLight())
                // setting the bot constrains (maxVel, maxAccel, maxAngVel track width)
                .setConstraints(60,60,Math.toRadians(180),Math.toRadians(180), 15).build();
        secondbot.runAction(secondbot.getDrive().actionBuilder(new Pose2d(-24, 69, Math.toRadians(270)))
                .build());

        //some settings
        mm.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(firstbot)
                .addEntity(secondbot)
                .start();
    }
}