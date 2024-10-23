package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);
        //Each square is 23 by 23 pixel thingies

        RoadRunnerBotEntity redBotRight = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 17)
                .setDimensions(17, 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(14, -61, Math.toRadians(90)))
                                //Outtakes preload
                                .lineToLinearHeading(new Pose2d(0, -34, Math.toRadians(90)))
                                .waitSeconds(1)
                               .lineToLinearHeading(new Pose2d(30, -34, Math.toRadians(90)))
                               // .waitSeconds(1)
                              //  .lineToLinearHeading(new Pose2d(-55, -58, Math.toRadians(-130)))
                             //   .waitSeconds(1)
                                .build()
                );

        RoadRunnerBotEntity redBotLeft = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 17)
                .setDimensions(17, 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-32, -61, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(0, -34, Math.toRadians(90)))
                                .waitSeconds(1)
                                .lineToLinearHeading(new Pose2d(23, -34, Math.toRadians(90)))
                                .waitSeconds(1)
                                .lineToLinearHeading(new Pose2d(-55, -58, Math.toRadians(-130)))
                                .waitSeconds(1)
                                .build()
                );

        RoadRunnerBotEntity blueBotLeft = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 17)
                .setDimensions(17, 15)
                .setColorScheme(new ColorSchemeBlueDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(10, 61, Math.toRadians(270)))
                                .lineToLinearHeading(new Pose2d(1, 33, Math.toRadians(-90)))
                                .waitSeconds(1)
                                .lineToLinearHeading(new Pose2d(-34, 33, Math.toRadians(-90)))
                                .waitSeconds(1)
                                .lineToLinearHeading(new Pose2d(56, 56, Math.toRadians(50)))
                                .waitSeconds(1)
                                .build()
                );

        RoadRunnerBotEntity blueBotRight = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(45, 45, Math.toRadians(180), Math.toRadians(180), 17)
                .setDimensions(17, 15)
                .setColorScheme(new ColorSchemeBlueDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-32, 61, Math.toRadians(270)))
                                //Mats are 23'23
                                //Outtakes preload
                                .lineToLinearHeading(new Pose2d(1, 33, Math.toRadians(-90)))
                                .waitSeconds(1)
                                //Outtakes preload
                                .lineToLinearHeading(new Pose2d(-34, 33, Math.toRadians(-90)))
                                .waitSeconds(1)
                                //right
                                .lineToLinearHeading(new Pose2d(56, 56, Math.toRadians(50)))
                                .waitSeconds(1)
                                .lineToLinearHeading(new Pose2d(-40, 33, Math.toRadians(270)))
                                .waitSeconds(1)
                                .lineToLinearHeading(new Pose2d(56, 56, Math.toRadians(50)))
                                .waitSeconds(1)
                                .lineToLinearHeading(new Pose2d(-45, 33, Math.toRadians(270)))
                                .waitSeconds(1)
                                .lineToLinearHeading(new Pose2d(56, 56, Math.toRadians(50)))
                                .waitSeconds(1)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
              //  .addEntity(blueBotRight)
              //  .addEntity(blueBotLeft)
              //  .addEntity(redBotLeft)
                .addEntity(redBotRight)
                .start();




        /*.lineToLinearHeading(new Pose2d(-10, 61, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(-10, 35, Math.toRadians(270)))
                .waitSeconds(10)
                //Goes to cycle
                .lineToLinearHeading(new Pose2d(-32, 35, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(-60, 35, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(55, 35, Math.toRadians(270)))
                .turn(Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(55, 55, Math.toRadians(0)))
                .build()

         */
    }


}