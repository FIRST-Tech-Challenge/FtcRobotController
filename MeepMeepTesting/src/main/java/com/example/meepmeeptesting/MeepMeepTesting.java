package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        final double robotCenterToArm = 10;
        MeepMeep meepMeep = new MeepMeep(700);
        Pose2d blueBasket = new Pose2d(64, 64,Math.toRadians(45));
        Pose2d redBasket = new Pose2d(-64, -64,Math.toRadians(225));
        Pose2d blueRobot2StartPosition = new Pose2d(-58, 64,Math.toRadians(270));
        Pose2d blueRobot1StartPosition = new Pose2d(60, 60,Math.toRadians(270));
        Pose2d redRobot1StartPosition = new Pose2d(-60,-60,Math.toRadians(90));
        Pose2d redRobot2StartPosition = new Pose2d(58,-64,Math.toRadians(90));
        Pose2d blueGamePiece1 = new Pose2d(-48+robotCenterToArm, 26,Math.toRadians(180));
        Pose2d blueGamePiece2 = new Pose2d(-58+robotCenterToArm, 26,Math.toRadians(180));
        Pose2d blueGamePiece3 = new Pose2d(-68+robotCenterToArm, 26,Math.toRadians(180));
        Pose2d yellow1GamePiece1 = new Pose2d(68, 26+robotCenterToArm,Math.toRadians(270));
        Pose2d yellow1GamePiece2 = new Pose2d(58, 26+robotCenterToArm,Math.toRadians(270));
        Pose2d yellow1GamePiece3 = new Pose2d(48, 26+robotCenterToArm,Math.toRadians(270));
        Pose2d redGamePiece1 = new Pose2d(68-robotCenterToArm, -26, Math.toRadians(0));
        Pose2d redGamePiece2 = new Pose2d(58-robotCenterToArm, -26, Math.toRadians(0));
        Pose2d redGamePiece3 = new Pose2d(48-robotCenterToArm, -26, Math.toRadians(0));
        Pose2d yellow2GamePiece1 = new Pose2d(-68, -26-robotCenterToArm,Math.toRadians(90));
        Pose2d yellow2GamePiece2 = new Pose2d(-58, -26-robotCenterToArm,Math.toRadians(90));
        Pose2d yellow2GamePiece3 = new Pose2d(-48, -26-robotCenterToArm,Math.toRadians(90));

        // Declare our first bot
        RoadRunnerBotEntity blue1Bot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        blue1Bot.runAction(blue1Bot.getDrive().actionBuilder(blueRobot1StartPosition)
                .strafeToLinearHeading(blueBasket.position,blueBasket.heading)
                .strafeToLinearHeading(yellow1GamePiece1.position,yellow1GamePiece1.heading)
                .strafeToLinearHeading(blueBasket.position,blueBasket.heading)
                .strafeToLinearHeading(yellow1GamePiece2.position,yellow1GamePiece2.heading)
                .strafeToLinearHeading(blueBasket.position,blueBasket.heading)
                .strafeToLinearHeading(yellow1GamePiece3.position,yellow1GamePiece3.heading)
                .strafeToLinearHeading(blueBasket.position,blueBasket.heading)

//                .splineToLinearHeading(blueBasket,Math.toRadians(0))
//                .strafeToLinearHeading(yellow1GamePiece3.position,yellow1GamePiece3.heading)
//                .splineToLinearHeading(blueBasket,Math.toRadians(0))
//                .strafeToLinearHeading(yellow1GamePiece2.position,yellow1GamePiece2.heading)
//                .splineToLinearHeading(blueBasket,Math.toRadians(0))
//                .strafeToLinearHeading(yellow1GamePiece1.position,yellow1GamePiece1.heading)
//                .strafeToLinearHeading(blueBasket.position,blueBasket.heading)



                .build());

        // Declare out second bot
        RoadRunnerBotEntity blue2Bot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be red
                .setColorScheme(new ColorSchemeBlueLight())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        blue2Bot.runAction(blue2Bot.getDrive().actionBuilder(blueRobot2StartPosition)
//                .strafeToLinearHeading(basket.position,basket.heading)
                .splineToLinearHeading(blueBasket, Math.toRadians(22.5))
                .strafeToLinearHeading(blueGamePiece1.position,blueGamePiece1.heading)
                .strafeToLinearHeading(blueBasket.position,blueBasket.heading)
                .strafeToLinearHeading(blueGamePiece2.position,blueGamePiece2.heading)
                .strafeToLinearHeading(blueBasket.position,blueBasket.heading)
                .strafeToLinearHeading(blueGamePiece3.position,blueGamePiece3.heading)
                .strafeToLinearHeading(blueBasket.position,blueBasket.heading)
                .build());

        RoadRunnerBotEntity red2Bot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        red2Bot.runAction(red2Bot.getDrive().actionBuilder(redRobot2StartPosition)
                        //.waitSeconds(1)
//                .strafeToLinearHeading(redBasket.position,redBasket.heading)
                        .splineToLinearHeading(redBasket, Math.toRadians(22.5))
                        .strafeToLinearHeading(redGamePiece3.position,redGamePiece3.heading)
                .strafeToLinearHeading(redBasket.position,redBasket.heading)
                .strafeToLinearHeading(redGamePiece2.position,redGamePiece2.heading)
                .strafeToLinearHeading(redBasket.position,redBasket.heading)
                .strafeToLinearHeading(redGamePiece1.position,redGamePiece1.heading)
                .build());

        RoadRunnerBotEntity red1Bot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedLight())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        red1Bot.runAction(red1Bot.getDrive().actionBuilder(redRobot1StartPosition)
                .strafeToLinearHeading(redBasket.position,redBasket.heading)
                .strafeToLinearHeading(yellow2GamePiece1.position,yellow2GamePiece1.heading)
                .strafeToLinearHeading(redBasket.position,redBasket.heading)
                .strafeToLinearHeading(yellow2GamePiece2.position,yellow2GamePiece2.heading)
                .strafeToLinearHeading(redBasket.position,redBasket.heading)
                .strafeToLinearHeading(yellow2GamePiece3.position,yellow2GamePiece3.heading)
                .strafeToLinearHeading(redBasket.position,redBasket.heading)
                .build()
        );

        RoadRunnerBotEntity blue3Bot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be red
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        blue3Bot.runAction(blue3Bot.getDrive().actionBuilder(blueRobot2StartPosition)
//                .strafeToLinearHeading(basket.position,basket.heading)
                .splineToLinearHeading(blueBasket, Math.toRadians(22.5))
                .strafeToLinearHeading(yellow1GamePiece1.position,yellow1GamePiece1.heading)
                .strafeToLinearHeading(blueBasket.position,blueBasket.heading)
                .strafeToLinearHeading(yellow1GamePiece2.position,yellow1GamePiece2.heading)
                .strafeToLinearHeading(blueBasket.position,blueBasket.heading)
                .strafeToLinearHeading(yellow1GamePiece3.position,yellow2GamePiece3.heading)
                .strafeToLinearHeading(blueBasket.position,blueBasket.heading)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                // Add both of our declared bot entities
                .addEntity(blue1Bot)
                .addEntity(blue2Bot)
                .addEntity(red2Bot)
                .addEntity(red1Bot)
                .addEntity(blue3Bot)
                .start();
    }
}