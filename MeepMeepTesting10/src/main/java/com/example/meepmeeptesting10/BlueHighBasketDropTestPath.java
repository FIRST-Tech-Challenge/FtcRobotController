package com.example.meepmeeptesting10;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class BlueHighBasketDropTestPath {

        public static void main(String[] args) {
            MeepMeep meepMeep = new MeepMeep(800);

            RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width

                    .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                    .build();

            DriveShim drive = myBot.getDrive();


        double xPositionInit = 0;
        double yPositionInit = 60.0;
        double headingInit = Math.toRadians(0.0);

        Pose2d initialPose = new Pose2d(xPositionInit, yPositionInit, headingInit);

        double xDestPositionDropSampleInHand = 48;
        double yDestPositionDropSampleInHand = 48;
        double headingDestPositionDropSampleInHand = Math.toRadians(0.0);

        TrajectoryActionBuilder initToHighBasket = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(xDestPositionDropSampleInHand, yDestPositionDropSampleInHand), Math.toRadians(45))
                .waitSeconds(1)
                .turnTo(Math.toRadians(-90))
                .lineToY(38)
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(xDestPositionDropSampleInHand, yDestPositionDropSampleInHand), Math.toRadians(45))
                .waitSeconds(1)
//                .turnTo(Math.toRadians(0))
                .strafeTo(new Vector2d(38, 24));
//                .strafeToLinearHeading(new Vector2d(xDestPositionDropSampleInHand, yDestPositionDropSampleInHand), Math.toRadians(0));

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();


                myBot.runAction(new SequentialAction(initToHighBasket.build()));

   }

}