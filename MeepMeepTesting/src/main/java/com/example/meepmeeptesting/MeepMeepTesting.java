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
        MeepMeep meepMeep = new MeepMeep(700);
        Pose2d blueBasket = new Pose2d(64, 64,Math.toRadians(45));
        Pose2d redBasket = new Pose2d(-64, -64,Math.toRadians(225));
        Pose2d blueRobot2StartPosition = new Pose2d(-58, 64,Math.toRadians(270));
        Pose2d blueRobot1StartPosition = new Pose2d(60, 60,Math.toRadians(270));
        Pose2d redRobot1StartPosition = new Pose2d(0,0,0);
        Pose2d redRobot2StartPosition = new Pose2d(0,0,0);
        Pose2d blueGamePiece1 = new Pose2d(-48, 26,Math.toRadians(180));
        Pose2d blueGamePiece2 = new Pose2d(-58, 26,Math.toRadians(180));
        Pose2d blueGamePiece3 = new Pose2d(-68, 26,Math.toRadians(180));
        Pose2d yellow1GamePiece1 = new Pose2d(68, 26,Math.toRadians(270));
        Pose2d yellow1GamePiece2 = new Pose2d(58, 26,Math.toRadians(270));
        Pose2d yellow1GamePiece3 = new Pose2d(48, 26,Math.toRadians(270));
        Pose2d redGamePiece1 = new Pose2d(68, -26, Math.toRadians(0));
        Pose2d redGamePiece2 = new Pose2d(58, -26, Math.toRadians(0));
        Pose2d redGamePiece3 = new Pose2d(48, -26, Math.toRadians(0));
        Pose2d yellow2GamePiece1 = new Pose2d(-68, -26,Math.toRadians(270));
        Pose2d yellow2GamePiece2 = new Pose2d(-58, -26,Math.toRadians(270));
        Pose2d yellow2GamePiece3 = new Pose2d(-48, -26,Math.toRadians(270));

        // Declare our first bot
        RoadRunnerBotEntity myFirstBot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myFirstBot.runAction(myFirstBot.getDrive().actionBuilder(blueRobot1StartPosition)
//                .strafeToLinearHeading(blueBasket.position,blueBasket.heading)
//                .strafeToLinearHeading(yellow1GamePiece1.position,yellow1GamePiece1.heading)
//                .strafeToLinearHeading(blueBasket.position,blueBasket.heading)
//                .strafeToLinearHeading(yellow1GamePiece2.position,yellow1GamePiece2.heading)
//                .strafeToLinearHeading(blueBasket.position,blueBasket.heading)
//                .strafeToLinearHeading(yellow1GamePiece3.position,yellow1GamePiece3.heading)

                .splineToLinearHeading(blueBasket,Math.toRadians(0))
                .strafeToLinearHeading(yellow1GamePiece3.position,yellow1GamePiece3.heading)
                .splineToLinearHeading(blueBasket,Math.toRadians(0))
                .strafeToLinearHeading(yellow1GamePiece2.position,yellow1GamePiece2.heading)
                .splineToLinearHeading(blueBasket,Math.toRadians(0))
                .strafeToLinearHeading(yellow1GamePiece1.position,yellow1GamePiece1.heading)
                .strafeToLinearHeading(blueBasket.position,blueBasket.heading)



                .build());

        // Declare out second bot
        RoadRunnerBotEntity mySecondBot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be red
                .setColorScheme(new ColorSchemeBlueLight())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        mySecondBot.runAction(mySecondBot.getDrive().actionBuilder(blueRobot2StartPosition)
//                .strafeToLinearHeading(basket.position,basket.heading)
                .splineToLinearHeading(blueBasket, Math.toRadians(22.5))
                .strafeToLinearHeading(blueGamePiece1.position,blueGamePiece1.heading)
                .strafeToLinearHeading(blueBasket.position,blueBasket.heading)
                .strafeToLinearHeading(blueGamePiece2.position,blueGamePiece2.heading)
                .strafeToLinearHeading(blueBasket.position,blueBasket.heading)
                .strafeToLinearHeading(blueGamePiece3.position,blueGamePiece3.heading)
                .strafeToLinearHeading(blueBasket.position,blueBasket.heading)
                .build());

        RoadRunnerBotEntity myThirdBot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myThirdBot.runAction(myFirstBot.getDrive().actionBuilder(new Pose2d(23.7, -69, Math.toRadians(90)))
                .waitSeconds(1)
//                .strafeToLinearHeading(redBasket.position,redBasket.heading)
                .splineToLinearHeading(redBasket, Math.toRadians(22.5))
                .strafeToLinearHeading(redGamePiece3.position,redGamePiece3.heading)
                .strafeToLinearHeading(redBasket.position,redBasket.heading)
                .strafeToLinearHeading(redGamePiece2.position,redGamePiece2.heading)
                .strafeToLinearHeading(redBasket.position,redBasket.heading)
                .strafeToLinearHeading(redGamePiece1.position,redGamePiece1.heading)
                .build());

        RoadRunnerBotEntity myForthBot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedLight())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myForthBot.runAction(myForthBot.getDrive().actionBuilder(new Pose2d(-23.7, -69, Math.toRadians(90)))

                .build()
        );
        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                // Add both of our declared bot entities
                .addEntity(myFirstBot)
                .addEntity(mySecondBot)
                .addEntity(myThirdBot)
                .addEntity(myForthBot)
                .start();
    }
}