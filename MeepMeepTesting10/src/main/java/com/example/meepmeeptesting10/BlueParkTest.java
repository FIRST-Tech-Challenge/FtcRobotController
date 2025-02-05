package com.example.meepmeeptesting10;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class BlueParkTest {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width

                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 19)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(72, 0, Math.toRadians(-180)))
                .lineToX(30)
                .turn(Math.toRadians(-90))
                .lineToY(30)
                .turn(Math.toRadians(90))
                .lineToX(0)
                .turn(Math.toRadians(90))
                .lineToY(0)
                .turn(Math.toRadians(90))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();

                //BLUE BASKET STARTS HERE
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder
//                                        (new Pose2d(20, 65, Math.toRadians(-90)))
//
//
//                                //Path for Blue Basket Drop
//                                .lineToSplineHeading(new Pose2d(37, 37, Math.toRadians(-45)))
//                                .waitSeconds(0.5)
//                                .lineToSplineHeading(new Pose2d(53, 57, Math.toRadians(45)))
//                                .waitSeconds(1)
//                                .lineToSplineHeading(new Pose2d(47, 37, Math.toRadians(-45)))
//                                .waitSeconds(0.5)
//                                .lineToSplineHeading(new Pose2d(53, 57, Math.toRadians(45)))
//                                .waitSeconds(1)
//                                .lineToSplineHeading(new Pose2d(59, 37, Math.toRadians(-45)))
//                                .waitSeconds(0.5)
//                                .lineToSplineHeading(new Pose2d(53, 57, Math.toRadians(45)))
//                                .waitSeconds(1)
//                                .lineToSplineHeading(new Pose2d(25, 25, Math.toRadians(0)))
//
//                                /*
//                                This is Blue Basket Drop, but without Spline
//
//                                .forward(2)
//                                .turn(Math.toRadians(45))
//                                .forward(35)
//                                .turn(Math.toRadians(-45))
//                                .forward(-25)
//                                .turn(Math.toRadians(90))
//                                .turn(Math.toRadians(-70))
//                                .forward(30)
//                                .forward(-30)
//                                .turn(Math.toRadians(70))
//                                .turn(Math.toRadians(-60))
//                                .forward(30)
//                                .forward(-30)
//                                .turn(Math.toRadians(60))
//                                .turn(Math.toRadians(-110))
//                                .forward(50)
//                                */
//                                //BLUE BASKET ENDS HERE
//
//                                .build());
//
//
//        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
//                .setDarkMode(true)
//                .setBackgroundAlpha(0.95f)
//                .addEntity(myBot)
//                .start();
    }
}